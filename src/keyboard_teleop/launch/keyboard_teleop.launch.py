from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="keyboard_teleop",
            executable="keyboard_teleop_node",
            name="keyboard_teleop_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "joint_names": ["joint1", "joint2", "joint3", "joint4"],
                "joint_states_topic": "cone/joint_states",
                "joint_commands_topic": "cone/joint_commands",
                "step_rad": 0.03,      
                "poll_rate_hz": 50.0,
                "joint_min": [-1.5708, -1.5708, -1.5708, -3.1416],
                "joint_max": [ 1.5708,  1.5708,  1.5708,  3.1416],
            }],
        ),
    ])
