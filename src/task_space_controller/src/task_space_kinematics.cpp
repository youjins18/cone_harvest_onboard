#include "task_space_controller/task_space_kinematics.hpp"

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace task_space_controller
{

    TaskSpaceKinematics::TaskSpaceKinematics(const std::string &urdf_path)
    {
        model_.gravity = RigidBodyDynamics::Math::Vector3d(0., 0., -9.81);
        const bool ok = RigidBodyDynamics::Addons::URDFReadFromFile(
            urdf_path.c_str(), &model_, /*floating_base=*/false, /*verbose=*/false);
        if (!ok)
        {
            throw std::runtime_error("RBDL URDFReadFromFile failed: " + urdf_path);
        }

        joint_names_ = {"joint_x", "joint_y", "joint1", "joint2", "joint3", "joint4"};

        // 조인트 이름 -> URDF child link 이름 (URDF 구조에서 고정, 파싱 순서 가정 없음).
        // RBDL은 각 body를 URDF의 child link 이름으로 등록하므로, joint 이름이 아니라
        // 이 child link 이름으로 GetBodyId() 조회
        static const std::pair<const char *, const char *> kJointToChildLink[] = {
            {"joint_x", "carriage_x"},
            {"joint_y", "carriage_y"},
            {"joint1", "link1"},
            {"joint2", "link2"},
            {"joint3", "link3"},
            {"joint4", "link4"},
        };

        q_index_.assign(joint_names_.size(), -1);
        lower_.assign(joint_names_.size(), -std::numeric_limits<double>::infinity());
        upper_.assign(joint_names_.size(), std::numeric_limits<double>::infinity());

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const std::string child_link = kJointToChildLink[i].second;
            const unsigned int body_id = model_.GetBodyId(child_link.c_str());
            if (body_id == std::numeric_limits<unsigned int>::max())
            {
                throw std::runtime_error(
                    "RBDL: could not find link '" + child_link + "' for joint '" + joint_names_[i] + "'.");
            }
            q_index_[i] = model_.mJoints[body_id].q_index;
        }

        // URDF <limit> (RBDL URDF reader는 limit 정보를 보존하지 않음)
        lower_[0] = -0.5;
        upper_[0] = 0.5; // joint_x
        lower_[1] = -0.5;
        upper_[1] = 0.5; // joint_y
        lower_[2] = 0.0;
        upper_[2] = 1.5708; // joint1
        lower_[3] = -1.5708;
        upper_[3] = 1.5708; // joint2
        lower_[4] = -1.5708;
        upper_[4] = 1.5708; // joint3
        // joint4: continuous, lower_/upper_[5]는 +-inf 유지

        // URDF의 'tcp' fixed joint(link4 기준 xyz="0 0 0.25")를 그대로 사용하므로
        // TCP 오프셋을 코드에 별도로 하드코딩하지 않는다(RBDL은 fixed body id를 그대로 지원).
        tcp_body_id_ = model_.GetBodyId("tcp");
        if (tcp_body_id_ == std::numeric_limits<unsigned int>::max())
        {
            throw std::runtime_error("RBDL: could not find body 'tcp'.");
        }
    }

    int TaskSpaceKinematics::qIndexForJoint(const std::string &name) const
    {
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (joint_names_[i] == name)
            {
                return q_index_[i];
            }
        }
        return -1;
    }

    double TaskSpaceKinematics::lowerLimit(int local_index) const { return lower_[local_index]; }
    double TaskSpaceKinematics::upperLimit(int local_index) const { return upper_[local_index]; }

    double TaskSpaceKinematics::clamp(int local_index, double value) const
    {
        return std::min(std::max(value, lower_[local_index]), upper_[local_index]);
    }

    Eigen::Vector3d TaskSpaceKinematics::currentPosition(
        const RigidBodyDynamics::Math::VectorNd &Q) const
    {
        return RigidBodyDynamics::CalcBodyToBaseCoordinates(
            const_cast<RigidBodyDynamics::Model &>(model_), Q, tcp_body_id_, tcp_local_offset_, false);
    }

    Eigen::Matrix3d TaskSpaceKinematics::currentOrientation(
        const RigidBodyDynamics::Math::VectorNd &Q) const
    {
        // CalcBodyWorldOrientation은 base(world)-from-body 회전을 리턴하므로,
        // world 좌표계 기준 body 자세(R_world_body)를 얻으려면 전치해야 한다.
        const RigidBodyDynamics::Math::Matrix3d r_body_base = RigidBodyDynamics::CalcBodyWorldOrientation(
            const_cast<RigidBodyDynamics::Model &>(model_), Q, tcp_body_id_, false);
        return r_body_base.transpose();
    }

    Eigen::MatrixXd TaskSpaceKinematics::computeTaskJacobian(
        const RigidBodyDynamics::Math::VectorNd &Q) const
    {
        RigidBodyDynamics::Math::MatrixNd J =
            RigidBodyDynamics::Math::MatrixNd::Zero(6, model_.qdot_size);
        // RBDL 컨벤션: 위 3행 = angular, 아래 3행 = linear. error6([linear;angular])와 맞추기 위해 재정렬.
        RigidBodyDynamics::CalcPointJacobian6D(
            const_cast<RigidBodyDynamics::Model &>(model_), Q, tcp_body_id_, tcp_local_offset_, J, false);
        Eigen::MatrixXd j_reordered(6, model_.qdot_size);
        j_reordered.topRows(3) = J.bottomRows(3);
        j_reordered.bottomRows(3) = J.topRows(3);
        return j_reordered;
    }

    Eigen::VectorXd TaskSpaceKinematics::solveDampedLeastSquares(
        const Eigen::MatrixXd &J, const Eigen::VectorXd &error6, double lambda) const
    {
        const Eigen::MatrixXd jjt = J * J.transpose();
        const Eigen::MatrixXd damped =
            jjt + (lambda * lambda) * Eigen::MatrixXd::Identity(jjt.rows(), jjt.cols());
        const Eigen::VectorXd y = damped.ldlt().solve(error6);
        return J.transpose() * y;
    }

    RigidBodyDynamics::Math::VectorNd TaskSpaceKinematics::solveIK(
        const RigidBodyDynamics::Math::VectorNd &q_init,
        const Eigen::Vector3d &goal_pos, const Eigen::Matrix3d &goal_r,
        double damping_lambda, double max_joint_velocity,
        double *out_error_norm) const
    {
        constexpr int kMaxIters = 200;
        constexpr double kTol = 1e-6;
        constexpr double kIterDt = 0.05; // 반복 1회당 가상 시간 간격

        RigidBodyDynamics::Model &model = const_cast<RigidBodyDynamics::Model &>(model_);
        RigidBodyDynamics::Math::VectorNd q = q_init;
        double error_norm = 0.0;
        for (int iter = 0; iter < kMaxIters; ++iter)
        {
            RigidBodyDynamics::UpdateKinematicsCustom(model, &q, nullptr, nullptr);
            const Eigen::Vector3d cur_pos = currentPosition(q);
            const Eigen::Matrix3d cur_r = currentOrientation(q);
            const Eigen::VectorXd error6 = computeError6(goal_pos, goal_r, cur_pos, cur_r);
            error_norm = error6.norm();
            if (error_norm < kTol)
            {
                break;
            }

            const Eigen::MatrixXd j = computeTaskJacobian(q);
            Eigen::VectorXd qdot = solveDampedLeastSquares(j, error6, damping_lambda);
            qdot = qdot.cwiseMax(-max_joint_velocity).cwiseMin(max_joint_velocity);

            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                const int qi = q_index_[i];
                q[qi] = clamp(static_cast<int>(i), q[qi] + qdot[qi] * kIterDt);
            }
        }
        if (out_error_norm)
        {
            *out_error_norm = error_norm;
        }
        return q;
    }

    double quinticTimeScaling(double t_norm)
    {
        const double t = std::min(std::max(t_norm, 0.0), 1.0);
        return 6.0 * std::pow(t, 5) - 15.0 * std::pow(t, 4) + 10.0 * std::pow(t, 3);
    }

    Eigen::VectorXd computeError6(
        const Eigen::Vector3d &goal_pos, const Eigen::Matrix3d &goal_r,
        const Eigen::Vector3d &cur_pos, const Eigen::Matrix3d &cur_r)
    {
        const Eigen::Vector3d pos_err = goal_pos - cur_pos;

        const Eigen::Quaterniond q_goal(goal_r);
        const Eigen::Quaterniond q_cur(cur_r);
        Eigen::Quaterniond q_err = (q_goal * q_cur.conjugate()).normalized();
        if (q_err.w() < 0.0)
        {
            q_err.coeffs() *= -1.0; // 최단 경로(shortest path)가 되도록 부호 보정(이중 피복 처리)
        }
        const Eigen::Vector3d rot_err = 2.0 * q_err.vec();

        Eigen::VectorXd error6(6);
        error6.head<3>() = pos_err;
        error6.tail<3>() = rot_err;
        return error6;
    }

} // namespace task_space_controller
