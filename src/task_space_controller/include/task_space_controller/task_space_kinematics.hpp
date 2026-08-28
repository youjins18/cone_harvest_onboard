#ifndef TASK_SPACE_CONTROLLER__TASK_SPACE_KINEMATICS_HPP_
#define TASK_SPACE_CONTROLLER__TASK_SPACE_KINEMATICS_HPP_

#include <rbdl/rbdl.h>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace task_space_controller
{

    class TaskSpaceKinematics
    {
    public:
        // URDF 경로로 RBDL 모델을 로드 (floating_base=false). 실패 시 std::runtime_error.
        explicit TaskSpaceKinematics(const std::string &urdf_path);

        int numJoints() const { return static_cast<int>(joint_names_.size()); }
        const std::vector<std::string> &jointNames() const { return joint_names_; }

        // 조인트 이름 -> RBDL Q(및 QDot) 인덱스. 이 모델은 전부 1-DOF 조인트이므로 동일 인덱스 사용.
        // 찾지 못하면 -1.
        int qIndexForJoint(const std::string &name) const;

        double lowerLimit(int local_index) const;
        double upperLimit(int local_index) const;
        double clamp(int local_index, double value) const;

        // Q는 model().q_size 크기. 사전에 RigidBodyDynamics::UpdateKinematicsCustom() 호출 필요.
        Eigen::Vector3d currentPosition(const RigidBodyDynamics::Math::VectorNd &Q) const;
        Eigen::Matrix3d currentOrientation(const RigidBodyDynamics::Math::VectorNd &Q) const;

        // 6xN task Jacobian. 행 순서 = [linear(vx,vy,vz); angular(wx,wy,wz)].
        Eigen::MatrixXd computeTaskJacobian(const RigidBodyDynamics::Math::VectorNd &Q) const;

        // damped least squares: qdot = J^T (JJ^T + lambda^2 I)^-1 * error6
        Eigen::VectorXd solveDampedLeastSquares(
            const Eigen::MatrixXd &J, const Eigen::VectorXd &error6, double lambda) const;

        // 반복 DLS-IK: q_init에서 시작해 (goal_pos, goal_r)에 대응하는 조인트각을 구한다.
        // 최대 반복 횟수 내에 수렴하지 못해도 마지막 q를 그대로 반환하며(가장 가까운 근사해),
        // out_error_norm이 null이 아니면 마지막 반복의 오차 크기(error6.norm())를 채운다.
        RigidBodyDynamics::Math::VectorNd solveIK(
            const RigidBodyDynamics::Math::VectorNd &q_init,
            const Eigen::Vector3d &goal_pos, const Eigen::Matrix3d &goal_r,
            double damping_lambda, double max_joint_velocity,
            double *out_error_norm = nullptr) const;

        RigidBodyDynamics::Model &model() { return model_; }
        unsigned int tcpBodyId() const { return tcp_body_id_; }
        const Eigen::Vector3d &tcpLocalOffset() const { return tcp_local_offset_; }

    private:
        RigidBodyDynamics::Model model_;
        std::vector<std::string> joint_names_;
        std::vector<int> q_index_;
        std::vector<double> lower_, upper_;
        unsigned int tcp_body_id_ = 0;
        // tcp_body_id_가 URDF의 'tcp' body를 직접 가리키므로 추가 오프셋은 항상 0이다.
        Eigen::Vector3d tcp_local_offset_{0.0, 0.0, 0.0};
    };

    // ---- task-space 궤적/오차 계산에 쓰이는 공유 순수 함수 ----
    // (task_space_controller_node.cpp의 publishTimerCallback 및 solveIK 내부에서 사용)

    // quintic time scaling: t_norm(0~1 클램프) -> s. s(0)=0, s(1)=1, s'/s''는 양 끝에서 0.
    double quinticTimeScaling(double t_norm);

    // [position error; quaternion-based orientation error(2*sign(w)*vec(q_goal*q_cur^-1))] 6D 벡터.
    Eigen::VectorXd computeError6(
        const Eigen::Vector3d &goal_pos, const Eigen::Matrix3d &goal_r,
        const Eigen::Vector3d &cur_pos, const Eigen::Matrix3d &cur_r);

} // namespace task_space_controller

#endif // TASK_SPACE_CONTROLLER__TASK_SPACE_KINEMATICS_HPP_
