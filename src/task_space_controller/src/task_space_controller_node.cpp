// task_space_controller_node: 목표 pose를 구독해 task-space IK를 풀고, joint-space quintic
//   궤적으로 스트리밍하는 독립 ROS2 노드.
//   구독: <target_pose_topic> (geometry_msgs/PoseStamped) - 목표 pose. position은 절대 좌표,
//     orientation은 "지금 자세 기준 상대 델타 회전"을 나타내는 쿼터니언(yaw는 쓰지 않는 것을
//     전제로 한다).
//   구독: <joint_states_topic> (sensor_msgs/JointState) - 현재 조인트 위치(IK 시작점/궤적 시작점).
//   발행: <joint_trajectory_topic> (기본값 cone/joint_commands, sensor_msgs/JointState) - quintic
//     조인트공간 궤적을 sample_rate_hz 주기로 한 스텝씩 스트리밍(단일 trajectory_msgs 메시지로
//     묶지 않음). dynamixel_arm_driver가 쓰는 것과 동일한 "이름+위치" 단일 스텝 JointState
//     관례를 그대로 따른다.
#include "task_space_controller/task_space_kinematics.hpp"

#include <rbdl/rbdl.h>

#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

namespace task_space_controller
{

    namespace
    {
        constexpr double kIkConvergenceWarnTol = 1e-3;
    } // namespace

    class TaskSpaceControllerNode : public rclcpp::Node
    {
    public:
        TaskSpaceControllerNode()
            : rclcpp::Node("task_space_controller_node")
        {
            const std::string default_urdf_path =
                ament_index_cpp::get_package_share_directory("task_space_controller") +
                "/models/Forest_Cone_Harvesting_Robot_with_translation.urdf";
            const std::string urdf_path = declare_parameter("urdf_path", default_urdf_path);
            const std::string target_pose_topic = declare_parameter("target_pose_topic", std::string("cone/target_pose"));
            // dynamixel_arm_driver가 그대로 쓰는 cone/joint_states,cone/joint_commands에 맞춘 기본값
            const std::string joint_states_topic = declare_parameter("joint_states_topic", std::string("cone/joint_states"));
            const std::string joint_trajectory_topic =
                declare_parameter("joint_trajectory_topic", std::string("cone/joint_commands"));
            duration_ = declare_parameter("duration", 5.0);
            const double sample_rate_hz = declare_parameter("sample_rate_hz", 200.0);
            ik_damping_lambda_ = declare_parameter("ik_damping_lambda", 0.05);
            max_joint_velocity_ = declare_parameter("max_joint_velocity", 0.5);

            kin_ = std::make_unique<TaskSpaceKinematics>(urdf_path);
            q_current_ = RigidBodyDynamics::Math::VectorNd::Zero(kin_->model().q_size);

            joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_trajectory_topic, 10);
            joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                joint_states_topic, 10,
                std::bind(&TaskSpaceControllerNode::jointStatesCallback, this, std::placeholders::_1));
            target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                target_pose_topic, 10,
                std::bind(&TaskSpaceControllerNode::targetPoseCallback, this, std::placeholders::_1));

            const auto period = std::chrono::duration<double>(1.0 / sample_rate_hz);
            publish_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&TaskSpaceControllerNode::publishTimerCallback, this));

            RCLCPP_INFO(
                get_logger(), "task_space_controller_node ready (urdf: %s, joints: %zu)",
                urdf_path.c_str(), kin_->jointNames().size());
        }

    private:
        // 이름이 일치하는 조인트만 갱신하고 나머지는 이전 값을 유지(조인트 일부만 있어도 안전).
        void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k)
            {
                const int qi = kin_->qIndexForJoint(msg->name[k]);
                if (qi < 0)
                {
                    continue;
                }
                q_current_[qi] = msg->position[k];
            }
        }

        void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            RigidBodyDynamics::UpdateKinematicsCustom(kin_->model(), &q_current_, nullptr, nullptr);
            const Eigen::Matrix3d cur_r = kin_->currentOrientation(q_current_);

            const Eigen::Vector3d goal_pos(
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            const Eigen::Quaterniond delta_r(
                msg->pose.orientation.w, msg->pose.orientation.x,
                msg->pose.orientation.y, msg->pose.orientation.z);
            const Eigen::Matrix3d goal_r = cur_r * delta_r.normalized().toRotationMatrix();

            double error_norm = 0.0;
            const RigidBodyDynamics::Math::VectorNd q_goal = kin_->solveIK(
                q_current_, goal_pos, goal_r, ik_damping_lambda_, max_joint_velocity_, &error_norm);
            if (error_norm > kIkConvergenceWarnTol)
            {
                RCLCPP_WARN(
                    get_logger(), "IK did not converge (residual %.6f) -- moving to closest reachable pose",
                    error_norm);
            }

            traj_start_q_ = q_current_;
            traj_goal_q_ = q_goal;
            start_time_ = now();
            trajectory_active_ = true;
        }

        void publishTimerCallback()
        {
            if (!trajectory_active_)
            {
                return;
            }

            const double t_norm = (now() - start_time_).seconds() / duration_;
            const double s = quinticTimeScaling(std::clamp(t_norm, 0.0, 1.0));

            sensor_msgs::msg::JointState msg;
            msg.header.stamp = now();
            msg.name = kin_->jointNames();
            msg.position.resize(msg.name.size());
            for (size_t i = 0; i < msg.name.size(); ++i)
            {
                const int qi = kin_->qIndexForJoint(msg.name[i]);
                msg.position[i] = traj_start_q_[qi] + s * (traj_goal_q_[qi] - traj_start_q_[qi]);
            }
            joint_state_pub_->publish(msg);

            if (t_norm >= 1.0)
            {
                trajectory_active_ = false;
            }
        }

        std::unique_ptr<TaskSpaceKinematics> kin_;
        RigidBodyDynamics::Math::VectorNd q_current_;

        double duration_ = 5.0;
        double ik_damping_lambda_ = 0.05;
        double max_joint_velocity_ = 0.5;

        bool trajectory_active_ = false;
        rclcpp::Time start_time_;
        RigidBodyDynamics::Math::VectorNd traj_start_q_;
        RigidBodyDynamics::Math::VectorNd traj_goal_q_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
    };

} // namespace task_space_controller

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<task_space_controller::TaskSpaceControllerNode>());
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("task_space_controller_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
