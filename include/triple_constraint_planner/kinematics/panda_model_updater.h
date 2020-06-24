#pragma once

#include <Eigen/Dense>

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <rbdl/rbdl.h>
#include <iostream>
#include <memory>
using namespace Eigen;
using namespace RigidBodyDynamics;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

#define deg2rad(ang) ((ang)*M_PI / 180.0)
#define rad2deg(ang) ((ang)*180 / M_PI)
class FrankaModelUpdater
{
public:
    FrankaModelUpdater();
    void PandaRBDLModel();
    Isometry3d getTransform(const Vector7d &q);
    Matrix<double, 6, 7> getJacobian(const Vector7d &q);
    Matrix<double, 7, 7> getMassMatrix(const Vector7d &q);
    Matrix<double, 7, 1> getGravity(const Vector7d &q);
    
    std::shared_ptr<Model> rbdl_model_;

    // for robot model construction
    Model model_;
    unsigned int body_id_[7];
    Math::Vector3d com_position_[7];
    Math::Matrix3d inertia_[7];
    Vector3d joint_position_[7];
    Body body_[7];
    Joint joint_[7];

    // arm parameters --
    Eigen::Matrix<double, 7, 7> mass_matrix_; //m
    Eigen::Matrix<double, 7, 1> coriolis_;
    Eigen::Matrix<double, 7, 1> tau_measured_;     //torque
    Eigen::Matrix<double, 7, 1> tau_desired_read_; //torque_desired
    Eigen::Matrix<double, 7, 1> gravity_;          //g
    Eigen::Matrix<double, 6, 7> jacobian_;         //j
    Eigen::Matrix<double, 3, 1> position_;         //xp
    Eigen::Matrix<double, 3, 3> rotation_;         //xr
    Eigen::Isometry3d transform_;
    Eigen::Matrix<double, 6, 1> xd_; //xp_desired + xr_desired?

    Eigen::Matrix<double, 7, 1> initial_q_; ///< initial joint configuration for idle control
    Eigen::Isometry3d initial_transform_;     ///< initial transform for idle control
    bool idle_controlled_{false};           ///< it indicates that this arm is under the idle control status. that is FrankaModelUpdater has valid initial transform and is using the transform.
    bool target_updated_{false};            ///< it is used to check whether any other action is on going excep the idle control
    // -- arm parameters

    double delta_tau_max_{0.05};
};

class panda_ik
{
public:
    panda_ik();
    bool solve(VectorXd start, Isometry3d target, Eigen::Ref<Eigen::VectorXd> solution);
    Eigen::VectorXd getRandomConfig();
    bool randomSolve(Isometry3d target, Eigen::Ref<Eigen::VectorXd> solution);
    
    Eigen::Isometry3d fk(const Eigen::Ref<const Eigen::VectorXd> &q);
    Eigen::Isometry3d random_fk();

private:
    std::string chain_start{"panda_link0"};
    std::string chain_end{"panda_hand"};
    TRAC_IK::TRAC_IK tracik_solver;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    KDL::Chain chain;
    Eigen::VectorXd lb_, ub_;
    Eigen::VectorXd length;
};