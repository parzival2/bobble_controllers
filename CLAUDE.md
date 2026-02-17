# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 package (`bobble_controllers`) that implements a PID balance controller for the BobbleBot self-balancing robot. It reads EKF-filtered orientation from `bobble_localization` and commands wheel velocities via the diff_drive_controller in `bobble_description`.

## Build Commands

```bash
# Build the package (run from workspace root ~/Projects/ros_ws)
colcon build --packages-select bobble_controllers

# Source the workspace
source install/setup.bash
```

## Launch Commands

```bash
# Launch balance controller
ros2 launch bobble_controllers balance.launch.py

# Launch with custom namespace
ros2 launch bobble_controllers balance.launch.py namespace:=robot1

# Launch with real hardware (disable sim time)
ros2 launch bobble_controllers balance.launch.py use_sim_time:=false
```

### Launch Parameters (`balance.launch.py`)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_sim_time` | `true` | Use simulated time (set `false` for real hardware) |
| `namespace` | `bobble` | Robot namespace |

## Architecture

### Data Flow

```
/bobble/imu ──> [EKF] ──> /bobble/odom/filtered ──> [balance_node] ──> /bobble/cmd_vel ──> [diff_drive_controller] ──> wheels
                  ^                                       ^
/bobble_controller/odom ─┘          ~/velocity_cmd ───────┘  (teleop / velocity commands)
```

### Topics
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/bobble/odom/filtered` | `nav_msgs/Odometry` | Input | EKF-filtered orientation (from `bobble_localization`) |
| `~/velocity_cmd` | `geometry_msgs/Twist` | Input | Desired forward/turn velocity (from teleop) |
| `/bobble/cmd_vel` | `geometry_msgs/TwistStamped` | Output | Wheel velocity commands (to diff_drive_controller) |

### PID Controller (`balance_node.py`)

The controller extracts pitch from the EKF odometry quaternion and runs a PID loop to keep the robot upright. Forward velocity commands shift the pitch setpoint to lean the robot in the direction of travel.

**Safety features:**
- Fallen detection (pitch > 45 deg) — stops motors and resets integrator
- Auto-recovery when righted
- Anti-windup clamping on integral term
- dt guard against sim pause/unpause gaps
- Zero velocity on shutdown

### PID Tuning

All parameters are dynamically tunable at runtime:

```bash
ros2 param set /bobble/balance_node pid.kp 8.0
ros2 param set /bobble/balance_node pid.kd 0.3
ros2 param set /bobble/balance_node pid.ki 0.2
```

**Tuning procedure (Ziegler-Nichols):**
1. Set Ki=0, Kd=0
2. Increase Kp until sustained oscillation (this is Ku)
3. Set Kp = 0.6 * Ku
4. Add Kd for damping
5. Add small Ki for steady-state error correction

### Config Parameters (`config/balance_pid.yaml`)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `control_rate` | 50.0 Hz | Control loop rate (matches EKF output) |
| `pid.kp` | 5.0 | Proportional gain |
| `pid.ki` | 0.5 | Integral gain |
| `pid.kd` | 0.1 | Derivative gain |
| `pitch_setpoint` | 0.0 | Target pitch (rad) |
| `max_velocity` | 0.8 | Max wheel velocity (m/s) |
| `max_integral` | 0.5 | Anti-windup clamp |
| `fallen_threshold` | 0.7854 | ~45 deg safety cutoff (rad) |
| `velocity_to_pitch_gain` | 0.05 | Lean angle per m/s desired velocity |

### Teleop Integration

Use any standard ROS2 teleop node with topic remapping:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/bobble/balance_node/velocity_cmd
```

### Dependencies
- `rclpy` — ROS2 Python client
- `nav_msgs` — Odometry message type
- `geometry_msgs` — Twist/TwistStamped message types
- `std_msgs` — Standard messages

### Related Packages
- **`bobble_description`** — URDF, Gazebo simulation, diff_drive_controller
- **`bobble_localization`** — EKF sensor fusion providing `/bobble/odom/filtered`
