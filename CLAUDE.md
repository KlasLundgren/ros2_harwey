# CLAUDE.md — rules

These rules apply to every task in this project unless explicitly overridden.
Bias: caution over speed on non-trivial work. Use judgment on trivial tasks.

## Rule 1 — Think Before Coding
State assumptions explicitly. If uncertain, ask rather than guess.
Present multiple interpretations when ambiguity exists.
Push back when a simpler approach exists.
Stop when confused. Name what's unclear.

## Rule 2 — Simplicity First
Minimum code that solves the problem. Nothing speculative.
No features beyond what was asked. No abstractions for single-use code.
Test: would a senior engineer say this is overcomplicated? If yes, simplify.

## Rule 3 — Surgical Changes
Touch only what you must. Clean up only your own mess.
Don't "improve" adjacent code, comments, or formatting.
Don't refactor what isn't broken. Match existing style.

## Rule 4 — Goal-Driven Execution
Define success criteria. Loop until verified.
Don't follow steps. Define success and iterate.
Strong success criteria let you loop independently.

## Rule 5 — Use the model only for judgment calls
Use me for: classification, drafting, summarization, extraction.
Do NOT use me for: routing, retries, deterministic transforms.
If code can answer, code answers.

## Rule 6 — Token budgets are not advisory
Per-task: 4,000 tokens. Per-session: 30,000 tokens.
If approaching budget, summarize and start fresh.
Surface the breach. Do not silently overrun.

## Rule 7 — Surface conflicts, don't average them
If two patterns contradict, pick one (more recent / more tested).
Explain why. Flag the other for cleanup.
Don't blend conflicting patterns.

## Rule 8 — Read before you write
Before adding code, read exports, immediate callers, shared utilities.
"Looks orthogonal" is dangerous. If unsure why code is structured a way, ask.

## Rule 9 — Tests verify intent, not just behavior
Tests must encode WHY behavior matters, not just WHAT it does.
A test that can't fail when business logic changes is wrong.

## Rule 10 — Checkpoint after every significant step
Summarize what was done, what's verified, what's left.
Don't continue from a state you can't describe back.
If you lose track, stop and restate.

## Rule 11 — Match the codebase's conventions, even if you disagree
Conformance > taste inside the codebase.
If you genuinely think a convention is harmful, surface it. Don't fork silently.

## Rule 12 — Fail loud
"Completed" is wrong if anything was skipped silently.
"Tests pass" is wrong if any were skipped.
Default to surfacing uncertainty, not hiding it.

## Rule 13 — Update Docs
When a change in is made, updates relevant docs correspondingly so they always stay up to date with the lates changes.  

# CLAUDE.md — Harwey Backend

## Quick Reference

```bash
# Build
source /opt/ros/jazzy/setup.bash
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

ROS2 Jazzy project for an autonomous differential-drive agricultural robot (Raspberry Pi 5).

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

### Verification Before Done
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

## Core Principles

- **Simplicity First**: Make every change as simple as possible. Impact minimal code.
- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.
- **Minimal Impact**: Changes should only touch what's necessary. Avoid introducing bugs.

