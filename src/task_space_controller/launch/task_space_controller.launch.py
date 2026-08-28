from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("task_space_controller"),
                "models",
                "Forest_Cone_Harvesting_Robot_with_translation.urdf",
            ]
        ),
        description="Path to the URDF file used for RBDL-based task-space kinematics",
    )
    target_pose_topic_arg = DeclareLaunchArgument(
        "target_pose_topic", default_value="cone/target_pose",
        description="geometry_msgs/PoseStamped topic for task-space target pose requests",
    )
    joint_states_topic_arg = DeclareLaunchArgument(
        # dynamixel_arm_driver가 그대로 쓰는 cone/joint_states,cone/joint_commands에 맞춘 기본값
        # (remap 인자 없이 바로 연동되도록 온보드 배포본에서만 변경).
        "joint_states_topic", default_value="cone/joint_states",
        description="sensor_msgs/JointState topic providing the current joint configuration",
    )
    joint_trajectory_topic_arg = DeclareLaunchArgument(
        "joint_trajectory_topic", default_value="cone/joint_commands",
        description="sensor_msgs/JointState topic the streamed joint-space trajectory is published on",
    )
    duration_arg = DeclareLaunchArgument(
        "duration", default_value="5.0", description="Quintic trajectory duration [s]"
    )
    sample_rate_hz_arg = DeclareLaunchArgument(
        "sample_rate_hz", default_value="200.0", description="Trajectory streaming rate [Hz]"
    )
    ik_damping_lambda_arg = DeclareLaunchArgument(
        "ik_damping_lambda", default_value="0.05", description="DLS-IK damping factor"
    )
    max_joint_velocity_arg = DeclareLaunchArgument(
        "max_joint_velocity", default_value="0.5",
        description="Per-iteration joint velocity clamp for IK [rad/s]",
    )

    task_space_controller_node = Node(
        package="task_space_controller",
        executable="task_space_controller_node",
        output="screen",
        parameters=[
            {
                "urdf_path": LaunchConfiguration("urdf_path"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "joint_trajectory_topic": LaunchConfiguration("joint_trajectory_topic"),
                "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
                "sample_rate_hz": ParameterValue(
                    LaunchConfiguration("sample_rate_hz"), value_type=float
                ),
                "ik_damping_lambda": ParameterValue(
                    LaunchConfiguration("ik_damping_lambda"), value_type=float
                ),
                "max_joint_velocity": ParameterValue(
                    LaunchConfiguration("max_joint_velocity"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            urdf_path_arg,
            target_pose_topic_arg,
            joint_states_topic_arg,
            joint_trajectory_topic_arg,
            duration_arg,
            sample_rate_hz_arg,
            ik_damping_lambda_arg,
            max_joint_velocity_arg,
            task_space_controller_node,
        ]
    )
