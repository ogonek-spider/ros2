# CLAUDE.md — Spider Hexapod Robot (ROS2)

## Project Overview

A 6-legged hexapod robot control system built on ROS2. Supports both real hardware (serial) and Mujoco simulation. The robot has 18 revolute joints (3 per leg × 6 legs), controlled via ros2_control and a tripod gait locomotion engine.

## Package Structure

| Package | Type | Purpose |
|---------|------|---------|
| `spider_description` | ament_cmake | Robot URDF, meshes, Mujoco simulation models |
| `spider_ros_control` | ament_cmake (C++) | Hardware interface plugin + launch files |
| `spider_walker` | ament_python | Tripod gait engine + inverse kinematics |
| `spider_pid_tuning` | ament_python | Sine/square wave generators for PID tuning |

External submodules: `mujoco_ros2_control`, `urdf_parser_py`, `kdl_parser_py`, `pixi-robostack`

## Build & Run

```bash
# Enter Pixi dev environment (ROS2 Kilted)
cd pixi-robostack && pixi shell -e kilted

# Build all packages
colcon build
source install/local_setup.zsh

# Launch in simulation (Mujoco)
ros2 launch spider_ros_control spider-mujoco.launch.py

# Launch on real hardware
ros2 launch spider_ros_control spider.launch.py
```

## Robot Architecture

```
Joystick → teleop_twist_joy → /cmd_vel (Twist)
  → spider_walker (gait + IK)
  → JointTrajectoryController (ros2_control)
  → SpiderHardwareInterface (serial) OR Mujoco simulator
```

- **Control rate:** 100 Hz
- **Serial port:** `/dev/tty.usbmodem3101` at 230400 baud
- **Serial protocol library:** `twai_proto` (external, install separately)
- **Gait:** Tripod — Tripod 1: legs [1,3,5], Tripod 2: legs [2,4,6]

## Joint Naming Convention

Joints are named `{leg_id}-{motor_id}`:
- `1-1`, `1-2`, `1-3` ... `6-1`, `6-2`, `6-3`
- Motor order per leg: coxa → femur → tibia

## Kinematics

File: `spider_walker/spider_walker/kinematics_solver.py`

- **Leg dimensions:** coxa=0.25m, femur=0.669m, tibia=0.919m
- **Leg mount angles:** [-45°, 0°, +45°, 225°, 180°, 135°]
- **Neutral stance angles:** (0, 1.3, 1.65) rad per joint
- IK is solved numerically; FK/IK consistency tested in `spider_walker/test/test_kinematics.py`

## Key Files

| File | Description |
|------|-------------|
| `spider_ros_control/src/spider_hardware_interface.cpp` | C++ hardware plugin, serial I/O |
| `spider_ros_control/description/urdf/spider.urdf.xacro` | Real hardware ros2_control config |
| `spider_ros_control/description/urdf/spider-mujoco.urdf.xacro` | Mujoco sim config (PID: kp=7000, ki=1, kd=10) |
| `spider_ros_control/config/spider-controllers.yaml` | Controller config (100Hz, JointTrajectoryController) |
| `spider_ros_control/config/spider-teleop.yaml` | Joystick axis mappings |
| `spider_walker/spider_walker/spider_walker.py` | Main walker node (state machine) |
| `spider_walker/spider_walker/trajectory_generator.py` | Tripod gait trajectory builder |
| `spider_description/urdf/robot.urdf` | Full robot URDF (18 joints, STL meshes) |
| `spider_description/mujoco/scene.xml` | Mujoco world config |
| `spider_pid_tuning/spider_pid_tuning/sin_position_generator.py` | Single-joint sine/square wave test node |

## C++ Build Notes

- C++20 required
- Hardware interface compiles as a shared library (pluginlib plugin)
- Plugin declared in `spider_ros_control/spider_hardware_interface.xml`
- Key dependencies: `hardware_interface`, `rclcpp_lifecycle`, `twai_proto`

## Python Packages

Both `spider_walker` and `spider_pid_tuning` use `setup.py` (not `pyproject.toml`).

Entry points:
- `spider_walker`: `walker`, `ik`, `custom_ik`
- `spider_pid_tuning`: `singen`, `multigen`

## Testing

```bash
# Run kinematics unit tests
cd spider_walker && pytest test/test_kinematics.py
```

Tests cover: FK/IK consistency, leg stepping, mirror symmetry.

## URDF Generation

URDF was generated from Onshape CAD via `onshape-to-robot`. Do not hand-edit `robot.urdf` directly if the CAD model is updated — re-export instead.
