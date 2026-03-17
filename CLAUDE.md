# CLAUDE.md — Harwey Backend

## Quick Reference

```bash
# Build
source /opt/ros/humble/setup.bash
cd ros2_harwey-master
colcon build --symlink-install
source install/setup.bash

# Build single package
colcon build --packages-select navigation_core

# CAN setup (MUST run before motor control)
sudo ip link set can0 up type can bitrate 1000000

# Launch robot (real hardware)
ros2 launch navigation_core launch_robot_launch.py    # RSP + LIDAR
ros2 launch navigation_core diffbot.launch.py         # Motor control

# Navigation
ros2 launch navigation_core slam_launch.py            # SLAM mapping
ros2 launch navigation_core localization_launch.py map:=/path/to/map.yaml
ros2 launch navigation_core navigation_launch.py

# IMU
ros2 run imu_data imu_data_pub

# CAN diagnostics
candump can0
cansend can0 601#4041600000000000   # Read statusword node 1
```

## Architecture

ROS2 Humble project for an autonomous differential-drive agricultural robot (Raspberry Pi 5, Ubuntu 22.04).

**Packages:**
- `navigation_core` — Main package: URDF, launch files, config, and custom `DmkeCanHwInterface` (ros2_control plugin)
- `imu_data` — WT901 IMU serial reader → publishes `sensor_msgs/Imu`
- `nav2_rviz_plugins` — Custom RViz2 panels (Qt/C++)
- `motion_controller` — Stub (future use)
- `safety_supervisor` — Stub (future use)

**Data flow:** `/cmd_vel` → DiffDriveController → DmkeCanHwInterface → SocketCAN → DMKE drives

**Submodules:** `sllidar_ros2` (LIDAR driver). `ros2_canopen` listed but NOT used (replaced by custom SocketCAN).

## Key Files

| File | What it does |
|------|-------------|
| `src/navigation_core/src/dmke_can_hw_interface.cpp` | Custom ros2_control SystemInterface — CiA 402 over raw SocketCAN. This is the core hardware driver. |
| `src/imu_data/src/imu_data_pub.cpp` | Serial IMU reader (9600 baud, 11-byte packets, 0x55 sync) |
| `src/navigation_core/config/diff_controller.yaml` | Primary diff drive config (used with DmkeCanHwInterface) |
| `src/navigation_core/config/my_controllers.yaml` | Alternative config for ros2_canopen setup |
| `src/navigation_core/config/nav2_params.yaml` | Full Nav2 stack params (AMCL, MPPI, costmaps, docking) |
| `src/navigation_core/description/ros2_control.xacro` | Hardware params: CAN node IDs, ramp limits, following error |
| `src/navigation_core/launch/diffbot.launch.py` | Main motor control launch (controller_manager + spawners) |
| `src/navigation_core/launch/launch_robot_launch.py` | RSP + LIDAR launch |

## Hardware Specs

- **Motors:** 2x D80M-03230, 2500 PPR encoders (10000 counts/rev)
- **Gearbox:** RV50-30, ratio 1:50
- **Drives:** DMKE DCSV, CiA 402 profile, CAN node IDs: left=1, right=2
- **CAN bus:** 1 Mbit/s, standard frame, SDO communication (not PDO)
- **LIDAR:** SLAMTEC S3 on `/dev/ttyUSB0`
- **IMU:** WT901 9-axis on `/dev/ttyUSB0` (9600 baud)
- **Wheels:** 0.994m separation, 0.25m radius
- **Velocity limits:** 1.0 m/s linear, 2.0 rad/s angular

## Unit Conversions (in dmke_can_hw_interface.cpp)

- Position: 1 wheel rad = 79,577.47 encoder counts
- Velocity: 1 wheel rad/s = 795,774.72 drive velocity units
- These are derived from: `10000 counts/rev × 50 gear ratio / (2π)`

## Gotchas

- **CAN interface MUST be up before launching motor control** — `diffbot.launch.py` will fail silently if `can0` is not configured
- **Two config files exist** — `diff_controller.yaml` (for DmkeCanHwInterface, 50Hz, wheels 0.994m/0.25m) and `my_controllers.yaml` (for ros2_canopen, 100Hz, wheels 1.0m/0.2m). The active one depends on which launch file you use
- **The code comments say gear ratio 1:30 at the top of dmke_can_hw_interface.cpp but the actual constants use 1:50** — the constants were tuned empirically and verified with cansend tests
- **LIDAR and IMU may share `/dev/ttyUSB0`** — use udev rules to assign persistent names
- **SDO timeout is 100ms** — if drives are slow to respond, the control loop skips the read and uses last known value (non-blocking socket)
- **3-second delay in diffbot.launch.py** — controllers are spawned after a timer, not lifecycle events. If hardware init takes longer, increase the delay
- **Following error factory defaults are too tight** — DMKE drives default to 2500 counts / 10ms, which trips under normal load. The xacro sets 100,000 counts / 1000ms
- **Stale CAN frames after Ctrl+C** — the driver drains them on startup, but power-cycle drives if issues persist
- **`ros2_canopen` submodule exists but is NOT used** — replaced by custom SocketCAN due to build system conflicts

## Code Style

- C++ for hardware interfaces and RViz plugins (rclcpp, Qt)
- Python for launch files and utility scripts
- CMake build system (ament_cmake)
- Extensive inline `// WHY:` comments explain design decisions — maintain this pattern
- No unit tests currently exist

### Self-Improvement Loop
- After ANY correction from the user: update `tasks/lessons.md` with the pattern
- Write rules for yourself that prevent the same mistake
- Ruthlessly iterate on these lessons until mistake rate drops
- Review lessons at session start for relevant project

- ### Verification Before Done
- Never mark a task complete without proving it works
- Diff behavior between main and your changes when relevant
- Ask yourself: "Would a staff engineer approve this?"
- Run tests, check logs, demonstrate correctness

### Demand Elegance (Balanced)
- For non-trivial changes: pause and ask "is there a more elegant way?"
- If a fix feels hacky: "Knowing everything I know now, implement the elegant solution"
- Skip this for simple, obvious fixes — don't over-engineer
- Challenge your own work before presenting it

### Autonomous Bug Fixing
- When given a bug report: just fix it. Don't ask for hand-holding
- Point at logs, errors, failing tests — then resolve them
- Zero context switching required from the user
- Go fix failing CI tests without being told how

## Task Management

1. **Plan First**: Write plan to `tasks/todo.md` with checkable items
2. **Verify Plan**: Check in before starting implementation
3. **Track Progress**: Mark items complete as you go
4. **Explain Changes**: High-level summary at each step
5. **Document Results**: Add review section to `tasks/todo.md`
6. **Capture Lessons**: Update `tasks/lessons.md` after corrections

## Core Principles

- **Simplicity First**: Make every change as simple as possible. Impact minimal code.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.

