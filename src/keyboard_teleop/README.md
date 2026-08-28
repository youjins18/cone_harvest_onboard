# keyboard_teleop

4-DoF cone harvesting robot arm joint-jog teleop for ROS 2 Humble.

## Interface

- Subscribe: `/cone/joint_states` (`sensor_msgs/msg/JointState`)
- Publish: `/cone/joint_commands` (`sensor_msgs/msg/JointState`)
- Controlled joints: `joint1`, `joint2`, `joint3`, `joint4` only
- `carriage_x`, `carriage_y` are intentionally not used.

The node waits until feedback for all four joints has been received, then initializes its command target from the measured pose. With the current `dynamixel_arm_driver`, the arm pose at driver startup is defined as `[0, 0, 0, 0] rad`.

## Keys

| Key | Action |
|---|---|
| `q` / `a` | joint1 + / - |
| `w` / `s` | joint2 + / - |
| `e` / `d` | joint3 + / - |
| `r` / `f` | joint4 + / - |
| `h` | hold current measured pose |
| `0` | command startup-zero pose |
| `[` / `]` | decrease / increase step |
| `p` | print measured and target joint positions |
| `?` | help |
| `x` or `ESC` | hold and exit |

Default step: `0.03 rad` (~1.72 deg).

## Build

```bash
cd ~/Desktop/cone_harvest_onboard_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 launch dynamixel_arm_driver dynamixel_driver.launch.py
```

Terminal 2:

```bash
source ~/Desktop/cone_harvest_onboard_ws/install/setup.bash
ros2 run keyboard_teleop keyboard_teleop_node
```

Running with `ros2 run` is recommended because the node reads keyboard input directly from the terminal.

## Important

`task_space_controller` and `keyboard_teleop` both publish to `/cone/joint_commands`. Do not use them as command sources at the same time. For manual joint jogging, run `dynamixel_arm_driver + keyboard_teleop` and leave `task_space_controller` off.

The default joint limits are conservative relative limits around the driver-defined startup zero. Change `joint_min` / `joint_max` after verifying the actual mechanism limits.
