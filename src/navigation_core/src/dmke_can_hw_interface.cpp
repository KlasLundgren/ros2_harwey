// dmke_can_hw_interface.cpp
// Custom ros2_control SystemInterface for 2x DMKE DCSV CiA 402 drives
// over raw SocketCAN. Replaces ros2_canopen with direct frame I/O.
//
// WHY raw SocketCAN instead of ros2_canopen:
//   ros2_canopen's build system (cogen_dcf/generate_dcf/bus.yml) has
//   conflicting format requirements between its cmake macros and its
//   RobotSystem runtime parser. After hours of debugging, raw CAN is
//   simpler, fully transparent, and we already validated it with cansend/candump.
//
// Hardware:
//   - 2x DMKE DCSV drives, CiA 402 profile
//   - D80M-03230 motors, 2500 PPR encoders (10000 counts/rev)
//   - RV50-30 gearbox, ratio 1:30
//   - CAN bus at 1 Mbit/s, standard frame format
//
// Unit conversions (derived from tested cansend values):
//   Position: 1 wheel rad = 47746.48 encoder counts
//             (10000 counts/rev * 30 gear ratio) / (2*pi)
//   Velocity: 1 wheel rad/s = 477464.83 drive velocity units
//             (10000/60 * 10000 counts/rev/min-unit) * (30 gear) * (60/2pi)
//             Verified: 100 motor RPM = 166666 drive units (tested via cansend)

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>

namespace dmke_can_hw
{

// -- CiA 402 object dictionary addresses used by this driver --
// WHY these specific addresses: they're the minimum set needed for
// velocity mode operation, confirmed working via cansend/candump testing.
static constexpr uint16_t OD_CONTROLWORD       = 0x6040;
static constexpr uint16_t OD_STATUSWORD        = 0x6041;
static constexpr uint16_t OD_MODES_OF_OP       = 0x6060;
static constexpr uint16_t OD_TARGET_VELOCITY   = 0x60FF;
static constexpr uint16_t OD_ACTUAL_POSITION   = 0x6064;
static constexpr uint16_t OD_ACTUAL_VELOCITY   = 0x6069;  // DMKE-specific, not CiA 402 standard 0x606C

// CiA 402 controlword values
// WHY 0x0F for enable: the DMKE drives accept a single-step enable
// (skip ready-to-switch-on -> switched-on transitions). Tested and confirmed.
static constexpr uint16_t CW_SHUTDOWN          = 0x0006;
static constexpr uint16_t CW_SWITCH_ON         = 0x0007;
static constexpr uint16_t CW_ENABLE_OP         = 0x000F;
static constexpr uint16_t CW_DISABLE           = 0x0000;
static constexpr uint16_t CW_FAULT_RESET       = 0x0080;

// CiA 402 statusword bit masks
static constexpr uint16_t SW_READY_TO_SWITCH_ON = 0x0021;
static constexpr uint16_t SW_SWITCHED_ON        = 0x0023;
static constexpr uint16_t SW_OP_ENABLED         = 0x0027;
static constexpr uint16_t SW_FAULT              = 0x0008;
static constexpr uint16_t SW_STATE_MASK         = 0x006F;

// Velocity mode = 3
static constexpr uint8_t MODE_VELOCITY = 0x03;

// Unit conversion constants
// WHY these values: derived from encoder spec (2500 PPR = 10000 counts/rev)
// and gearbox ratio (1:30). Cross-checked against tested cansend values
// where 100 motor RPM = 166666 drive units.
static constexpr double COUNTS_PER_WHEEL_RAD = 79577.47;       // 10000 * 50 / (2*pi)
static constexpr double DRIVE_UNITS_PER_WHEEL_RAD_S = 795774.72; // 10000/60 * 10000 * 50 / (2*pi)

// Timeouts
static constexpr int SDO_TIMEOUT_MS = 100;
static constexpr int ENABLE_TIMEOUT_MS = 2000;


class DmkeCanHwInterface : public hardware_interface::SystemInterface
{
public:

  // ---- lifecycle: on_init ----
  // Opens SocketCAN socket and reads joint/CAN config from URDF parameters.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read CAN interface name from URDF hardware params, default "can0"
    can_interface_ = info.hardware_parameters.count("can_interface")
      ? info.hardware_parameters.at("can_interface")
      : "can0";

    // Read CAN node IDs from URDF hardware params
    // WHY per-joint node_id in hardware_parameters: avoids a separate YAML config
    // file. Two integers don't justify an external config dependency.
    if (info.hardware_parameters.count("left_node_id")) {
      node_ids_[0] = std::stoi(info.hardware_parameters.at("left_node_id"));
    }
    if (info.hardware_parameters.count("right_node_id")) {
      node_ids_[1] = std::stoi(info.hardware_parameters.at("right_node_id"));
    }

    // Validate joint count
    if (info.joints.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
        "Expected 2 joints (left, right), got %zu", info.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate interfaces: each joint needs velocity command + position & velocity state
    for (const auto & joint : info.joints) {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != "velocity") {
        RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
          "Joint '%s' must have exactly 1 command interface: velocity",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.state_interfaces.size() != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
          "Joint '%s' must have 2 state interfaces: position, velocity",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Open SocketCAN
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
        "Failed to open CAN socket: %s", strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
        "CAN interface '%s' not found: %s", can_interface_.c_str(), strerror(errno));
      close(sock_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
        "Failed to bind CAN socket: %s", strerror(errno));
      close(sock_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // WHY O_NONBLOCK: read() is called every control cycle (~10ms at 100Hz).
    // Blocking would stall the entire ros2_control update loop if a drive
    // doesn't respond. With nonblock we just skip and use the last known value.
    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

    // Set receive filter: accept SDO responses (0x580+id), boot-up (0x700+id),
    // and default TPDOs (0x180+id, 0x280+id) from both drives.
    // WHY include TPDOs: DMKE drives have default PDO mappings that send
    // unsolicited state-change frames. If we don't accept them they pile up
    // in the kernel buffer. We read and discard non-SDO frames in the SDO wait loop.
    struct can_filter rfilter[6];
    // Drive 1
    rfilter[0] = {0x580u + (uint32_t)node_ids_[0], CAN_SFF_MASK};  // SDO response
    rfilter[1] = {0x700u + (uint32_t)node_ids_[0], CAN_SFF_MASK};  // NMT boot-up
    rfilter[2] = {0x180u + (uint32_t)node_ids_[0], CAN_SFF_MASK};  // TPDO1
    // Drive 2
    rfilter[3] = {0x580u + (uint32_t)node_ids_[1], CAN_SFF_MASK};  // SDO response
    rfilter[4] = {0x700u + (uint32_t)node_ids_[1], CAN_SFF_MASK};  // NMT boot-up
    rfilter[5] = {0x180u + (uint32_t)node_ids_[1], CAN_SFF_MASK};  // TPDO1
    setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    // Drain any stale frames sitting in the socket buffer from a previous run.
    // WHY: if the previous process was killed (Ctrl+C) while drives were in
    // Operational state, they keep sending TPDOs. The kernel buffers those frames.
    // If we don't flush them, wait_sdo_response reads a stale TPDO instead of
    // the SDO response we're waiting for and times out.
    {
      struct can_frame drain{};
      int drained = 0;
      while (::read(sock_, &drain, sizeof(drain)) > 0) { drained++; }
      if (drained > 0) {
        RCLCPP_INFO(rclcpp::get_logger("DmkeCanHwInterface"),
          "Drained %d stale CAN frames from socket buffer", drained);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("DmkeCanHwInterface"),
      "CAN socket opened on %s, nodes %d/%d",
      can_interface_.c_str(), node_ids_[0], node_ids_[1]);

    return hardware_interface::CallbackReturn::SUCCESS;
  }


  // ---- State and command interface exports ----
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (size_t i = 0; i < 2; ++i) {
      interfaces.emplace_back(
        info_.joints[i].name, "position", &hw_positions_[i]);
      interfaces.emplace_back(
        info_.joints[i].name, "velocity", &hw_velocities_[i]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (size_t i = 0; i < 2; ++i) {
      interfaces.emplace_back(
        info_.joints[i].name, "velocity", &hw_commands_[i]);
    }
    return interfaces;
  }


  // ---- lifecycle: on_activate ----
  // Runs CiA 402 state machine to bring both drives to Operation Enabled.
  // WHY here and not on_init: ros2_control calls on_activate when controllers
  // are loaded. The drives should only be enabled when we're ready to command them.
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("DmkeCanHwInterface"),
      "Activating drives...");

    for (int i = 0; i < 2; ++i) {
      if (!enable_drive(i)) {
        RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
          "Failed to enable drive %d (node %d)", i, node_ids_[i]);
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("DmkeCanHwInterface"),
        "Drive %d (node %d) enabled in velocity mode", i, node_ids_[i]);
    }

    // Zero commands and read initial position
    hw_commands_[0] = 0.0;
    hw_commands_[1] = 0.0;

    return hardware_interface::CallbackReturn::SUCCESS;
  }


  // ---- lifecycle: on_deactivate ----
  // Sends zero velocity then disables drives.
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    RCLCPP_INFO(rclcpp::get_logger("DmkeCanHwInterface"),
      "Deactivating drives...");

    for (int i = 0; i < 2; ++i) {
      // Zero velocity first
      sdo_write_i32(node_ids_[i], OD_TARGET_VELOCITY, 0, 0);
      // Disable
      sdo_write_u16(node_ids_[i], OD_CONTROLWORD, 0, CW_DISABLE);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  // ---- read: called every control cycle ----
  // Reads position and velocity from both drives via SDO.
  // WHY SDO and not PDO: SDO is request-response, simpler to implement.
  // PDO requires mapping setup via SDO first anyway. For 2 motors at 100Hz,
  // the 4 SDO round-trips per cycle (~4ms total at 1Mbit) are well within budget.
  // If we later need >200Hz, switch to PDO.
  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    for (int i = 0; i < 2; ++i) {
      int32_t pos_raw = 0;
      int32_t vel_raw = 0;

      if (sdo_read_i32(node_ids_[i], OD_ACTUAL_POSITION, 0, pos_raw)) {
        hw_positions_[i] = static_cast<double>(pos_raw) / COUNTS_PER_WHEEL_RAD;
      }
      // else: keep last known value (nonblocking, drive might be slow)

      if (sdo_read_i32(node_ids_[i], OD_ACTUAL_VELOCITY, 0, vel_raw)) {
        hw_velocities_[i] = static_cast<double>(vel_raw) / DRIVE_UNITS_PER_WHEEL_RAD_S;
      }
    }
    return hardware_interface::return_type::OK;
  }


  // ---- write: called every control cycle ----
  // Sends velocity commands to both drives via SDO.
  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    for (int i = 0; i < 2; ++i) {
      int32_t vel_raw = static_cast<int32_t>(
        hw_commands_[i] * DRIVE_UNITS_PER_WHEEL_RAD_S);
      sdo_write_i32(node_ids_[i], OD_TARGET_VELOCITY, 0, vel_raw);
    }
    return hardware_interface::return_type::OK;
  }


  ~DmkeCanHwInterface()
  {
    if (sock_ >= 0) {
      close(sock_);
    }
  }


private:

  // ---- SDO write: 2 bytes (uint16) ----
  // SDO expedited write, 2 data bytes. CiA 301 command specifier = 0x2B.
  bool sdo_write_u16(int node_id, uint16_t index, uint8_t subindex, uint16_t value)
  {
    struct can_frame frame{};
    frame.can_id  = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x2B;  // write 2 bytes
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = value & 0xFF;
    frame.data[5] = (value >> 8) & 0xFF;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (::write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
      RCLCPP_WARN(rclcpp::get_logger("DmkeCanHwInterface"),
        "SDO write failed to node %d, index 0x%04X", node_id, index);
      return false;
    }
    return wait_sdo_response(node_id);
  }

  // ---- SDO write: 4 bytes (int32) ----
  // SDO expedited write, 4 data bytes. CiA 301 command specifier = 0x23.
  bool sdo_write_i32(int node_id, uint16_t index, uint8_t subindex, int32_t value)
  {
    struct can_frame frame{};
    frame.can_id  = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x23;  // write 4 bytes
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    std::memcpy(&frame.data[4], &value, 4);

    if (::write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
      RCLCPP_WARN(rclcpp::get_logger("DmkeCanHwInterface"),
        "SDO write failed to node %d, index 0x%04X", node_id, index);
      return false;
    }
    return wait_sdo_response(node_id);
  }

  // ---- SDO write: 1 byte (uint8) ----
  // SDO expedited write, 1 data byte. CiA 301 command specifier = 0x2F.
  bool sdo_write_u8(int node_id, uint16_t index, uint8_t subindex, uint8_t value)
  {
    struct can_frame frame{};
    frame.can_id  = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x2F;  // write 1 byte
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = value;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (::write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
      RCLCPP_WARN(rclcpp::get_logger("DmkeCanHwInterface"),
        "SDO write failed to node %d, index 0x%04X", node_id, index);
      return false;
    }
    return wait_sdo_response(node_id);
  }

  // ---- SDO read: 4 bytes (int32) ----
  // SDO upload initiate. CiA 301 command specifier = 0x40.
  bool sdo_read_i32(int node_id, uint16_t index, uint8_t subindex, int32_t & out)
  {
    struct can_frame frame{};
    frame.can_id  = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x40;  // read request
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;

    if (::write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
      return false;
    }

    struct can_frame resp{};
    if (!wait_sdo_response(node_id, &resp)) {
      return false;
    }

    // Check for abort (command byte 0x80)
    if (resp.data[0] == 0x80) {
      return false;
    }

    // Expedited response: data in bytes 4-7
    std::memcpy(&out, &resp.data[4], 4);
    return true;
  }

  // ---- SDO read: 2 bytes (uint16) ----
  bool sdo_read_u16(int node_id, uint16_t index, uint8_t subindex, uint16_t & out)
  {
    struct can_frame frame{};
    frame.can_id  = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x40;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;

    if (::write(sock_, &frame, sizeof(frame)) != sizeof(frame)) {
      return false;
    }

    struct can_frame resp{};
    if (!wait_sdo_response(node_id, &resp)) {
      return false;
    }

    if (resp.data[0] == 0x80) {
      return false;
    }

    std::memcpy(&out, &resp.data[4], 2);
    return true;
  }


  // ---- Wait for SDO response with timeout ----
  // WHY poll-based with timeout: the socket is nonblocking. We spin-read
  // for up to SDO_TIMEOUT_MS. In the enable sequence this is fine (startup only).
  // In the control loop, a missed read just returns false and we keep last value.
  bool wait_sdo_response(int node_id, struct can_frame * out_frame = nullptr)
  {
    uint32_t expected_id = 0x580 + node_id;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(SDO_TIMEOUT_MS);

    while (std::chrono::steady_clock::now() < deadline) {
      struct can_frame frame{};
      ssize_t nbytes = ::read(sock_, &frame, sizeof(frame));
      if (nbytes == sizeof(frame) && frame.can_id == expected_id) {
        if (out_frame) {
          *out_frame = frame;
        }
        // Check for SDO abort
        if (frame.data[0] == 0x80) {
          uint32_t abort_code;
          std::memcpy(&abort_code, &frame.data[4], 4);
          RCLCPP_WARN(rclcpp::get_logger("DmkeCanHwInterface"),
            "SDO abort from node %d: 0x%08X", node_id, abort_code);
          return false;
        }
        return true;
      }
      // Small sleep to avoid 100% CPU spin
      usleep(100);  // 0.1ms
    }

    RCLCPP_WARN(rclcpp::get_logger("DmkeCanHwInterface"),
      "SDO timeout waiting for node %d (0x%03X)", node_id, expected_id);
    return false;
  }


  // ---- CiA 402 drive enable sequence ----
  // Transitions: (any state) -> fault reset -> shutdown -> switch on -> enable operation
  // WHY this full sequence: some drives need explicit state transitions even though
  // writing 0x0F directly worked in cansend testing. Doing the full sequence is
  // more robust across power cycles and fault recovery.
  bool enable_drive(int drive_index)
  {
    int node_id = node_ids_[drive_index];

    // 0. Reset node first, then start it.
    // WHY reset before start: if the previous process was killed (Ctrl+C),
    // the drive is still in Operational state, potentially still enabled and
    // sending PDOs. NMT Reset puts it back to Pre-Operational with a clean
    // state machine. Then NMT Start transitions to Operational for SDO access.
    {
      struct can_frame nmt{};
      nmt.can_id  = 0x000;
      nmt.can_dlc = 2;

      // Reset Node
      nmt.data[0] = 0x81;
      nmt.data[1] = static_cast<uint8_t>(node_id);
      ::write(sock_, &nmt, sizeof(nmt));
      usleep(200000);  // 200ms for reset to complete

      // Drain frames generated during reset (boot-up message, etc)
      struct can_frame drain{};
      while (::read(sock_, &drain, sizeof(drain)) > 0) {}

      // Start Node
      nmt.data[0] = 0x01;
      nmt.data[1] = static_cast<uint8_t>(node_id);
      ::write(sock_, &nmt, sizeof(nmt));
      usleep(100000);  // 100ms for node to enter Operational

      // Drain boot-up frame sent after entering Operational
      while (::read(sock_, &drain, sizeof(drain)) > 0) {}

      RCLCPP_INFO(rclcpp::get_logger("DmkeCanHwInterface"),
        "Sent NMT Reset + Start to node %d", node_id);
    }

    // 1. Check current status
    uint16_t status = 0;
    if (!sdo_read_u16(node_id, OD_STATUSWORD, 0, status)) {
      RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
        "Cannot read statusword from node %d", node_id);
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("DmkeCanHwInterface"),
      "Node %d initial status: 0x%04X", node_id, status);

    // 2. If fault, reset it
    if (status & SW_FAULT) {
      RCLCPP_WARN(rclcpp::get_logger("DmkeCanHwInterface"),
        "Node %d in fault, resetting...", node_id);
      sdo_write_u16(node_id, OD_CONTROLWORD, 0, CW_FAULT_RESET);
      usleep(50000);  // 50ms for fault clear
      sdo_write_u16(node_id, OD_CONTROLWORD, 0, 0x0000);
      usleep(50000);
    }

    // 3. Set velocity mode (0x03)
    if (!sdo_write_u8(node_id, OD_MODES_OF_OP, 0, MODE_VELOCITY)) {
      RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
        "Failed to set velocity mode on node %d", node_id);
      return false;
    }

    // 4. Shutdown (transition to Ready To Switch On)
    sdo_write_u16(node_id, OD_CONTROLWORD, 0, CW_SHUTDOWN);
    usleep(20000);  // 20ms

    // 5. Switch On
    sdo_write_u16(node_id, OD_CONTROLWORD, 0, CW_SWITCH_ON);
    usleep(20000);

    // 6. Enable Operation
    sdo_write_u16(node_id, OD_CONTROLWORD, 0, CW_ENABLE_OP);
    usleep(20000);

    // 7. Verify we reached Operation Enabled
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(ENABLE_TIMEOUT_MS);
    while (std::chrono::steady_clock::now() < deadline) {
      if (sdo_read_u16(node_id, OD_STATUSWORD, 0, status)) {
        if ((status & SW_STATE_MASK) == SW_OP_ENABLED) {
          // Set initial velocity to zero
          sdo_write_i32(node_id, OD_TARGET_VELOCITY, 0, 0);
          return true;
        }
      }
      usleep(50000);  // 50ms between polls
    }

    RCLCPP_ERROR(rclcpp::get_logger("DmkeCanHwInterface"),
      "Node %d failed to reach Operation Enabled, final status: 0x%04X",
      node_id, status);
    return false;
  }


  // ---- Member variables ----
  int sock_ = -1;
  std::string can_interface_ = "can0";
  int node_ids_[2] = {1, 2};  // CAN node IDs, left=1, right=2

  // ros2_control state/command storage
  double hw_positions_[2]  = {0.0, 0.0};
  double hw_velocities_[2] = {0.0, 0.0};
  double hw_commands_[2]   = {0.0, 0.0};
};

}  // namespace dmke_can_hw

PLUGINLIB_EXPORT_CLASS(
  dmke_can_hw::DmkeCanHwInterface,
  hardware_interface::SystemInterface)