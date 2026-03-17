# Harwey — Autonomous Agricultural Robot

Navigation and control system for a differential-drive robot designed for autonomous harrowing operations in rectangular fields. Built on ROS2 Humble, targeting Raspberry Pi 5 with Ubuntu 22.04 LTS.

## Table of Contents

- [System Architecture](#system-architecture)
- [Hardware Overview](#hardware-overview)
- [ROS2 Packages](#ros2-packages)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Configuration](#configuration)
- [Launching](#launching)
- [Simulation](#simulation)
- [ROS2 Topics & Interfaces](#ros2-topics--interfaces)
- [Hardware Interface Details](#hardware-interface-details)
- [Project Structure](#project-structure)
- [Troubleshooting](#troubleshooting)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Nav2 Stack                              │
│  (AMCL / Cartographer, MPPI Controller, NavFn Planner, BT)      │
│                            │                                    │
│                       /cmd_vel                                  │
│                            ▼                                    │
│                 DiffDriveController                             │
│          (wheel separation, radius, limits)                     │
│                   │               │                             │
│             left rad/s       right rad/s                        │
│                   ▼               ▼                             │
│          ┌─────────────────────────────┐                        │
│          │   DmkeCanHwInterface        │                        │
│          │   (ros2_control plugin)     │                        │
│          │   CiA 402 over SocketCAN    │                        │
│          └──────────┬──────────────────┘                        │
│                     │ CAN bus (1 Mbit/s)                        │
│              ┌──────┴──────┐                                    │
│              ▼             ▼                                    │
│         DMKE Drive 1  DMKE Drive 2                              │
│         (Node ID 1)   (Node ID 2)                               │
│         Left wheel    Right wheel                               │
└─────────────────────────────────────────────────────────────────┘

Sensors:
  SLAMTEC S3 LIDAR ──► /scan (LaserScan)
  WT901 9-axis IMU ──► /imu_data (Imu)
  Wheel encoders   ──► /odom (Odometry, via DiffDriveController)
```

**Data flow summary:**

1. **Sensors** (LIDAR, IMU, encoders) publish data to ROS2 topics
2. **Nav2** uses sensor data for localization (AMCL) and path planning (NavFn + MPPI)
3. **DiffDriveController** converts `/cmd_vel` (m/s, rad/s) to per-wheel velocities (rad/s)
4. **DmkeCanHwInterface** converts wheel rad/s to drive velocity units and sends CAN frames
5. **Encoder feedback** flows back through the same chain for odometry

---

## Hardware Overview

### Drive System

| Component | Specification |
|-----------|--------------|
| Motors | 2x D80M-03230 |
| Encoders | 2500 PPR (10,000 counts/rev) |
| Gearbox | RV50-30, ratio 1:50 |
| Motor drives | 2x DMKE DCSV, CiA 402 profile |
| CAN bus | 1 Mbit/s, standard frame format |
| CAN node IDs | Left = 1, Right = 2 |
| Max motor speed | 3000 RPM |
| Max wheel speed | ~6.28 rad/s (3000 RPM / 50 gear ratio) |

### Chassis

| Parameter | Value |
|-----------|-------|
| Wheel separation (center-to-center) | 0.994 m |
| Wheel radius | 0.25 m |
| Max linear velocity (configured) | 1.0 m/s |
| Max angular velocity (configured) | 2.0 rad/s |
| Max linear acceleration | 0.5 m/s² |
| Max angular acceleration | 1.0 rad/s² |

### Sensors

| Sensor | Interface | Details |
|--------|-----------|---------|
| SLAMTEC S3 LIDAR | USB serial (`/dev/ttyUSB0`) | 360° laser scanner, driven by `sllidar_ros2` package |
| WT901 9-axis IMU | USB serial (`/dev/ttyUSB0`) | 9600 baud, 11-byte packets with 0x55 sync byte. Outputs accelerometer (±16g), gyroscope (±2000°/s), and orientation (Euler angles) |

### Compute

| Component | Specification |
|-----------|--------------|
| Platform | Raspberry Pi 5 |
| OS | Ubuntu 22.04 LTS |
| IP address (Ethernet) | 192.168.100.1/24 |

---

## ROS2 Packages

| Package | Description |
|---------|-------------|
| `navigation_core` | Main package: URDF/xacro robot model, launch files, config files, and the custom `DmkeCanHwInterface` hardware plugin |
| `imu_data` | IMU serial reader node — parses WT901 packets and publishes `sensor_msgs/Imu` |
| `motion_controller` | Motion controller (stub — reserved for future use) |
| `safety_supervisor` | Safety monitoring (stub — reserved for future use) |
| `nav2_rviz_plugins` | Custom RViz2 panels for Nav2 visualization (docking panel, goal tools, route tools) |

### External Dependencies (submodules)

| Package | Source |
|---------|--------|
| `sllidar_ros2` | Git submodule — SLAMTEC LIDAR driver |
| `ros2_canopen` | Listed in `dependencies.repos` (not actively used — replaced by custom SocketCAN interface) |

---

## Prerequisites

### System Requirements

- Ubuntu 22.04 LTS (on Raspberry Pi 5 or development machine)
- ROS2 Humble (full desktop or ros-base)

### ROS2 Dependencies

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-tf2-ros \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-teleop-twist-joy \
  ros-humble-joy \
  qtbase5-dev
```

### CAN Interface Setup

The CAN bus must be brought up before launching any motor control nodes:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

To make this persistent across reboots, add a systemd service or udev rule. Verify the interface is up:

```bash
ip link show can0
# Should show: can0: <NOARP,UP,LOWER_UP,ECHO> ...
```

---

## Building

```bash
# Navigate to workspace root
cd ros2_harwey-master

# Source ROS2
source /opt/ros/humble/setup.bash

# Initialize submodules (LIDAR driver)
git submodule update --init --recursive

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

To build a specific package:

```bash
colcon build --packages-select navigation_core
```

---

## Configuration

### Controller Configuration

**`config/diff_controller.yaml`** — Primary controller config used with the custom DMKE hardware interface:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `update_rate` | 50 Hz | Control loop rate. Set to 50 (not 100) because SDO round-trips take ~1-2ms each, and 4 reads + 2 writes per cycle = ~8-12ms |
| `wheel_separation` | 0.994 m | Center-to-center distance between wheels |
| `wheel_radius` | 0.25 m | Wheel radius at gearbox output |
| `linear.x.max_velocity` | 1.0 m/s | Conservative limit (theoretical max ~1.57 m/s) |
| `angular.z.max_velocity` | 2.0 rad/s | Conservative limit (theoretical max ~3.16 rad/s) |
| `cmd_vel_timeout` | 0.5 s | Wheels stop if no command received |
| `open_loop` | false | Uses encoder feedback for odometry |

**`config/my_controllers.yaml`** — Alternative config for ros2_canopen-based setup (includes `Cia402RobotController`). Uses 100 Hz update rate, 1.0 m wheel separation, and 0.2 m wheel radius.

### Hardware Parameters (URDF)

Configured in `description/ros2_control.xacro`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can0` | SocketCAN interface name |
| `left_node_id` | 1 | CAN node ID for left drive |
| `right_node_id` | 2 | CAN node ID for right drive |
| `speed_accel` | 1000 | Speed acceleration ramp (×1000 counts/s²) |
| `speed_decel` | 1000 | Speed deceleration ramp (×1000 counts/s²) |
| `speed_estop_decel` | 10000 | E-stop deceleration ramp |
| `following_error_window` | 100,000 | Position deviation tolerance (counts) |
| `following_error_timeout` | 1000 | Time before fault on deviation (ms) |

### Navigation Parameters

**`config/nav2_params.yaml`** — Full Nav2 stack configuration:

- **Localization:** AMCL with 500–2000 particles, likelihood field model
- **Path planner:** NavFn (Dijkstra by default, A* optional)
- **Path controller:** MPPI Controller (DiffDrive motion model)
- **Local costmap:** 3×3 m rolling window, 0.05 m resolution, voxel layer + inflation
- **Global costmap:** Static map layer + obstacle layer + inflation
- **Behaviors:** Spin, backup, drive on heading, wait, assisted teleop
- **Goal tolerance:** 0.25 m (xy), 0.25 rad (yaw)
- **Collision monitor:** Footprint approach action with LIDAR scan source
- **Docking server:** SimpleChargingDock plugin, 50 Hz controller frequency

### SLAM Configuration

**`config/mapper_params_online_async.yaml`** — Cartographer SLAM for online mapping
**`config/mapper_params_localization.yaml`** — Cartographer in localization-only mode (with existing map)

### Joystick Configuration

**`config/joystick.yaml`** — Gamepad button/axis mappings for manual teleop

---

## Launching

### Robot Startup (Real Hardware)

**Step 1:** Bring up the CAN interface:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

**Step 2:** Launch the robot state publisher and LIDAR:
```bash
ros2 launch navigation_core launch_robot_launch.py
```

**Step 3:** Launch the differential drive controller (motor control):
```bash
ros2 launch navigation_core diffbot.launch.py
```

This launches `controller_manager` with the `DmkeCanHwInterface` plugin, then spawns `diffbot_base_controller` and `joint_state_broadcaster` after a 3-second delay.

### Navigation

**With an existing map (localization + navigation):**
```bash
ros2 launch navigation_core localization_launch.py map:=/path/to/map.yaml
ros2 launch navigation_core navigation_launch.py
```

**SLAM mode (build a new map while navigating):**
```bash
ros2 launch navigation_core slam_launch.py
ros2 launch navigation_core navigation_launch.py
```

### Manual Control

**Joystick teleop:**
```bash
ros2 launch navigation_core joystick_launch.py
```

**Keyboard teleop:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### IMU Data

```bash
ros2 run imu_data imu_data_pub
```

---

## Simulation

A Gazebo simulation environment is provided for testing without physical hardware.

### Starting Simulation (Without Launch Files)

```bash
# 1. Launch Gazebo with the SLU simulation world
ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:=src/navigation_core/worlds/slu_simulation.sdf

# 2. Launch robot state publisher (new terminal)
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro simulation/robot_models/robot.urdf.xacro)"

# 3. Spawn the robot (new terminal)
ros2 launch ros_gz_sim gz_spawn_model.launch.py \
  world:=src/navigation_core/worlds/slu_simulation.sdf \
  topic:=robot_description \
  entity_name:=my_vehicle \
  x:=5.0 y:=5.0 z:=0.5
```

### Pre-saved Map Data

- `SLU_sim_save.yaml` / `SLU_sim_save.pgm` — Pre-built occupancy grid map
- `SLU_waypoints.yaml` — Predefined waypoint set for the simulation world

---

## ROS2 Topics & Interfaces

### Published Topics

| Topic | Message Type | Source | Rate |
|-------|-------------|--------|------|
| `/imu_data` | `sensor_msgs/msg/Imu` | `imu_data_pub` node | ~100 Hz (10ms timer) |
| `/scan` | `sensor_msgs/msg/LaserScan` | `sllidar_ros2` driver | Sensor rate |
| `/odom` | `nav_msgs/msg/Odometry` | `diffbot_base_controller` | 50 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` | 50 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher`, `diffbot_base_controller` | Continuous |

### Subscribed Topics

| Topic | Message Type | Consumer |
|-------|-------------|----------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `diffbot_base_controller` |

### TF Tree

```
map ─► odom ─► base_link ─► base_footprint
                   ├──► laser_frame
                   ├──► imu_link
                   ├──► left_wheel_joint
                   └──► right_wheel_joint
```

### Nav2 Action Servers

| Action | Type | Description |
|--------|------|-------------|
| `navigate_to_pose` | `NavigateToPose` | Navigate to a single goal pose |
| `navigate_through_poses` | `NavigateThroughPoses` | Navigate through multiple waypoints |
| `dock_robot` | `DockRobot` | Dock at charging station |
| `undock_robot` | `UndockRobot` | Undock from charging station |

---

## Hardware Interface Details

### DmkeCanHwInterface

Custom `ros2_control` `SystemInterface` plugin that replaces `ros2_canopen` with direct SocketCAN communication. Located in `src/navigation_core/src/dmke_can_hw_interface.cpp`.

**Why raw SocketCAN instead of ros2_canopen:** The ros2_canopen build system (`cogen_dcf`/`generate_dcf`/`bus.yml`) had conflicting format requirements between its CMake macros and runtime parser. Raw CAN is simpler, fully transparent, and was validated with `cansend`/`candump`.

### CiA 402 State Machine

The drive enable sequence follows the CiA 402 profile:

```
NMT Reset → NMT Start → Fault Reset (if needed) → Set Velocity Mode
→ Set Ramp Parameters → Set Following Error Limits
→ Shutdown → Switch On → Enable Operation
```

### Unit Conversions

| Conversion | Formula | Value |
|------------|---------|-------|
| Position: wheel rad → encoder counts | `10000 × 50 / (2π)` | 79,577.47 counts/rad |
| Velocity: wheel rad/s → drive units | `(10000/60 × 10000 × 50) × (60/2π)` | 795,774.72 units/(rad/s) |

### CAN Object Dictionary

| Address | Name | Size | Description |
|---------|------|------|-------------|
| `0x6040` | Control Word | 2 bytes | Drive control commands |
| `0x6041` | Status Word | 2 bytes | Drive status feedback |
| `0x6060` | Modes of Operation | 1 byte | Set to 3 (velocity mode) |
| `0x60FF` | Target Velocity | 4 bytes | Velocity command in drive units |
| `0x6064` | Actual Position | 4 bytes | Encoder position in counts |
| `0x6069` | Actual Velocity | 4 bytes | DMKE-specific velocity feedback |
| `0x2100` | Speed Accel Ramp | 4 bytes | Acceleration limit (×1000 counts/s²) |
| `0x2101` | Speed Decel Ramp | 4 bytes | Deceleration limit |
| `0x2102` | E-Stop Decel Ramp | 4 bytes | Emergency stop deceleration |
| `0x6067` | Following Error Window | 4 bytes | Position deviation tolerance (counts) |
| `0x6068` | Following Error Timeout | 2 bytes | Time before fault (ms) |

### SDO Communication

All drive communication uses SDO (Service Data Objects) — request/response protocol:

- **SDO request:** CAN ID `0x600 + node_id`
- **SDO response:** CAN ID `0x580 + node_id`
- **Timeout:** 100 ms per transaction
- **Socket:** Non-blocking to prevent control loop stalls
- **Throughput:** 4 reads + 2 writes per cycle = ~8-12 ms at 1 Mbit/s

---

## Project Structure

```
ros2_harwey-master/
├── README.md                          # This file
├── dependencies.repos                 # VCS repository imports
├── SLU_sim_save.yaml                  # Pre-built simulation map metadata
├── SLU_sim_save.pgm                   # Pre-built simulation map image
├── SLU_waypoints.yaml                 # Simulation waypoints
├── docs/
│   └── Architecture.drawio            # System architecture diagram
├── scripts/
│   ├── imuData.py                     # IMU data testing utility
│   └── calibrateImu.py                # IMU calibration utility
└── src/
    ├── navigation_core/               # Main robot package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── dmke_can_hw_plugin.xml      # Plugin export for ros2_control
    │   ├── src/
    │   │   └── dmke_can_hw_interface.cpp  # Custom CAN hardware interface
    │   ├── config/
    │   │   ├── diff_controller.yaml       # Diff drive controller params
    │   │   ├── my_controllers.yaml        # Alternative ros2_canopen config
    │   │   ├── nav2_params.yaml           # Full Nav2 stack params
    │   │   ├── joystick.yaml              # Gamepad button mappings
    │   │   ├── mapper_params_online_async.yaml  # Cartographer SLAM config
    │   │   └── mapper_params_localization.yaml  # Cartographer localization
    │   ├── description/
    │   │   ├── robot.urdf.xacro           # Main robot model (top-level)
    │   │   ├── robot_core.xacro           # Chassis, wheels, casters
    │   │   ├── ros2_control.xacro         # Hardware interface definition
    │   │   ├── lidar.xacro                # LIDAR mount and frame
    │   │   ├── inertial_macros.xacro      # Inertia calculation helpers
    │   │   └── gazebo_control.xacro       # Simulation control plugin
    │   ├── launch/
    │   │   ├── launch_robot_launch.py     # RSP + LIDAR startup
    │   │   ├── diffbot.launch.py          # Diff drive controller startup
    │   │   ├── rsp_launch.py              # Robot state publisher
    │   │   ├── navigation_launch.py       # Nav2 stack
    │   │   ├── slam_launch.py             # Cartographer SLAM
    │   │   ├── localization_launch.py     # AMCL localization
    │   │   ├── joystick_launch.py         # Joystick teleop
    │   │   └── can_control_launch.py      # CAN bus control
    │   └── worlds/
    │       └── slu_simulation.sdf         # Gazebo simulation world
    ├── imu_data/                       # IMU sensor package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src/
    │       └── imu_data_pub.cpp           # Serial IMU reader + ROS2 publisher
    ├── motion_controller/              # (Stub)
    ├── safety_supervisor/              # (Stub)
    ├── nav2_rviz_plugins/              # Custom RViz2 panels
    │   ├── src/                           # Qt/C++ panel implementations
    │   └── include/                       # Header files
    ├── ros2_canopen/                   # CANopen submodule (not actively used)
    └── sllidar_ros2/                   # LIDAR driver submodule
```

---

## Troubleshooting

### CAN Interface Not Found

```
CAN interface 'can0' not found: No such device
```

**Fix:** Bring up the CAN interface before launching:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

If the CAN adapter is not detected at all, check `dmesg | grep can` for hardware issues.

### SDO Timeout Errors

```
SDO timeout waiting for node X (0x58X)
```

**Possible causes:**
- Drive not powered on
- Wrong CAN node ID (check DIP switches on DMKE drives)
- CAN bus wiring issue (termination resistors, loose connections)
- CAN bitrate mismatch (must be 1 Mbit/s)

**Diagnostics:**
```bash
# Listen to CAN traffic
candump can0

# Send a test SDO read (statusword of node 1)
cansend can0 601#4041600000000000
# Expected response on 581#
```

### Drive Faults (Following Error)

```
SDO abort from node X: 0x...
```

The drive faults if actual position deviates from target by more than the configured following error window. This can happen under heavy mechanical load or if the gearbox is binding.

**Fix:** Increase `following_error_window` in `ros2_control.xacro` or reduce acceleration ramps.

### Serial Port Issues (IMU / LIDAR)

```
Error opening serial port /dev/ttyUSB0
```

**Fix:**
```bash
# Check which USB devices are connected
ls -la /dev/ttyUSB*

# Add user to dialout group (for serial access without sudo)
sudo usermod -a -G dialout $USER
# Log out and back in for group change to take effect
```

If both LIDAR and IMU use USB serial, they may be assigned to different `/dev/ttyUSBx` devices. Use udev rules to assign persistent device names.

### Stale CAN Frames After Ctrl+C

If the previous process was killed abruptly, the drives may still be sending TPDO frames. The `DmkeCanHwInterface` drains stale frames on startup automatically. If issues persist, power-cycle the drives.

### Controller Spawner Fails

```
Could not contact service /controller_manager/...
```

The `diffbot.launch.py` uses a 3-second delay before spawning controllers. If the hardware interface takes longer to initialize (e.g., drive enable timeout), increase the `TimerAction` period in the launch file.
