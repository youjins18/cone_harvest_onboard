from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    usb_port_arg = DeclareLaunchArgument(
        "usb_port",
        default_value="/dev/dynamixel",
        description="Dynamixel USB-serial adapter device path",
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="4500000",
        description="Serial baudrate (must match motors' configured baudrate)",
    )
    joint_names_arg = DeclareLaunchArgument(
        "joint_names",
        default_value="['joint1', 'joint2', 'joint3', 'joint4']",
        description="URDF joint names, ordered to match motor_ids",
    )
    motor_ids_arg = DeclareLaunchArgument(
        "motor_ids",
        default_value="[1, 2, 3, 4]",
        description="Dynamixel IDs, ordered to match joint_names",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="50.0",
        description="cone/joint_states publish rate",
    )

    dynamixel_driver_node = Node(
        package="dynamixel_arm_driver",
        executable="dynamixel_driver_node",
        output="screen",
        parameters=[{
            "usb_port": LaunchConfiguration("usb_port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "joint_names": LaunchConfiguration("joint_names"),
            "motor_ids": LaunchConfiguration("motor_ids"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
        }],
    )

    return LaunchDescription([
        usb_port_arg,
        baudrate_arg,
        joint_names_arg,
        motor_ids_arg,
        publish_rate_arg,
        dynamixel_driver_node,
    ])
