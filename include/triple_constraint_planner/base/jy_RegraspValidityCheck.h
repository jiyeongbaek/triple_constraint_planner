#pragma once

#include <sstream>
#include <string>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/collision_detection/collision_tools.h>

#include <geometry_msgs/Pose.h>

#include <triple_constraint_planner/kinematics/panda_model_updater.h>
#include <mutex>

namespace ob = ompl::base;
using namespace std;
using namespace Eigen;
class RgpValidityChecker
{
public:
    RgpValidityChecker()
    {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model = robot_model_loader.getModel();
        planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
        acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(planning_scene->getAllowedCollisionMatrix());
        robot_state::RobotState &current_state = planning_scene->getCurrentStateNonConst();

        Eigen::VectorXd default_start(7);
        default_start << 0, -0.785, 0, -1.571, 0, 1.571, 0.785;
        current_state.setJointGroupPositions("panda_1", default_start);
        current_state.setJointGroupPositions("panda_2", default_start);
        current_state.setJointGroupPositions("panda_3", default_start);
        current_state.update();

        planning_group = robot_model->getJointModelGroup("panda_arms");

        stefan_obj.header.frame_id = "base";
        stefan_obj.id = "stefan";

        shapes::Mesh *m = shapes::createMeshFromResource("file:///home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/assembly.stl");
        shape_msgs::Mesh stefan_mesh;
        shapes::ShapeMsg stefan_mesh_msg;
        shapes::constructMsgFromShape(m, stefan_mesh_msg);
        stefan_mesh = boost::get<shape_msgs::Mesh>(stefan_mesh_msg);
        stefan_obj.meshes.push_back(stefan_mesh);

        geometry_msgs::Pose stefan_pose;
        stefan_pose.orientation.w = 1.0;
        stefan_pose.position.x = 1.15;
        stefan_pose.position.y = -0.1;
        stefan_pose.position.z = 1.0;
        stefan_obj.mesh_poses.push_back(stefan_pose);
        stefan_obj.operation = stefan_obj.ADD;

        moveit_scene.is_diff = true;
        planning_scene->processCollisionObjectMsg(stefan_obj);

        for (int i = 1; i < 4; i++)
        {
            acm_->setEntry("stefan", "panda_" + std::to_string(i) + "_hand", true);
            acm_->setEntry("stefan", "panda_" + std::to_string(i) + "_leftfinger", true);
            acm_->setEntry("stefan", "panda_" + std::to_string(i) + "_rightfinger", true);
        }
        panda_ik_solver = std::make_shared<panda_ik>();

        base_1st = Eigen::Isometry3d::Identity();
        base_2nd = Eigen::Isometry3d::Identity();
        base_3rd = Eigen::Isometry3d::Identity();
        base_1st.translation() = Eigen::Vector3d(0, 0.3, 0.6);
        base_2nd.translation() = Eigen::Vector3d(0, -0.3, 0.6);
        base_3rd.translation() = Eigen::Vector3d(1.6, 0.0, 0.6);
        base_3rd.linear() << -1, 0, 0,
            0, -1, 0,
            0, 0, 1;
        Vector3d z_offset(0, 0, -0.103);
        for (int i = 0; i < 3; i++)
        {
            Sobj_grasp[i].setIdentity();
            Gobj_grasp[i].setIdentity();
        }
        //start
        Sobj_grasp[0].linear() = Quaterniond(-0.0265275, 0.706609, -0.0265278, 0.706609).toRotationMatrix();
        Sobj_grasp[0].translation() = Vector3d(-0.432292, 0.3779282, 0.28538042) + Sobj_grasp[0].linear() * z_offset;

        Sobj_grasp[1].linear() = Quaterniond(-0.0540426, -0.0582587, 0.67793, 0.730818).toRotationMatrix();
        Sobj_grasp[1].translation() = Vector3d(-0.6601151, -0.02137197, 0.02044866) + Sobj_grasp[1].linear() * z_offset;

        Sobj_grasp[2].linear() = Quaterniond(0.0102825, 0.707032, -0.707032, 0.0102825).toRotationMatrix();
        Sobj_grasp[2].translation() = Vector3d(-0.133369, 0.26831887, 0.01643235) + Sobj_grasp[2].linear() * z_offset;

        //goal
        Gobj_grasp[0].linear() = Quaterniond(0.497631, 0.483366, 0.516627, 0.501814).toRotationMatrix();
        Gobj_grasp[0].translation() = Vector3d(-0.4258375, 0.21016605, 0.0207994) + Gobj_grasp[0].linear() * z_offset;

        Gobj_grasp[1].linear() = Quaterniond(-0.0373972, 0.996136, 0.079409, 0.00298125).toRotationMatrix();
        Gobj_grasp[1].translation() = Vector3d(-0.7898177, -0.06157755, 0.3247905) + Gobj_grasp[1].linear() * z_offset;

        Gobj_grasp[2].linear() = Quaterniond(0.507219, 0.492676, -0.507217, -0.492677).toRotationMatrix();
        Gobj_grasp[2].translation() = Vector3d(-0.128369, 0.20230575, 0.017412) + Gobj_grasp[2].linear() * z_offset;
    }

    bool STreeValid(const ob::State *state, bool start = true)
    {
        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        Isometry3d base_obj;
        base_obj.setIdentity();
        base_obj.translation() = Vector3d(pos->values[0], pos->values[1], pos->values[2]);
        base_obj.linear() = Quaterniond(rot->w, rot->x, rot->y, rot->z).toRotationMatrix();

        Isometry3d target[3];
        for (int i = 0; i < 3; i++)
            target[i].setIdentity();
        if (start)
        {
            target[0] = base_1st.inverse() * base_obj * Sobj_grasp[0];
            target[1] = base_2nd.inverse() * base_obj * Sobj_grasp[1];
            target[2] = base_3rd.inverse() * base_obj * Sobj_grasp[2];
        }
        else
        {
            target[0] = base_1st.inverse() * base_obj * Gobj_grasp[0];
            target[1] = base_2nd.inverse() * base_obj * Gobj_grasp[1];
            target[2] = base_3rd.inverse() * base_obj * Gobj_grasp[2];
        }

        stefan_obj.meshes.clear();
        stefan_obj.mesh_poses.clear();
        geometry_msgs::Pose stefan_pose;
        Eigen::Quaterniond quat(base_obj.linear());
        stefan_pose.orientation.x = quat.x();
        stefan_pose.orientation.y = quat.y();
        stefan_pose.orientation.z = quat.z();
        stefan_pose.orientation.w = quat.w();

        stefan_pose.position.x = base_obj.translation()[0];
        stefan_pose.position.y = base_obj.translation()[1];
        stefan_pose.position.z = base_obj.translation()[2];
        stefan_obj.mesh_poses.push_back(stefan_pose);
        stefan_obj.operation = stefan_obj.MOVE;
        planning_scene->processCollisionObjectMsg(stefan_obj);

        // planning_scene->getCollisionWorld()->checkWorldCollision();
        if (planning_scene->isStateColliding("base"))
        {
            std::cout << "This configuration is colliding" << std::endl;
            return false;
        }

        // Eigen::VectorXd sol(21);
        bool success[3]{false};
        robot_state::RobotState robot_state = planning_scene->getCurrentState();

        for (int i = 0; i < 3; i++)
        {
            int iter = 0;
            while (iter < 10)
            {
                Eigen::Matrix<double, 7, 1> temp_sol;
                if (panda_ik_solver->randomSolve(target[i], temp_sol))
                {
                    robot_state.setJointGroupPositions("panda_" + std::to_string(i + 1), temp_sol);
                    robot_state.update();
                    collision_detection::CollisionRequest req;
                    // req.verbose = true;
                    req.group_name = "panda_" + std::to_string(i + 1);
                    collision_detection::CollisionResult res;
                    planning_scene->checkCollision(req, res, robot_state, *acm_);
                    if (!res.collision)
                    {
                        success[i] = true;
                        break;
                    }
                }
                ++iter;
            }
        }

        // planning_scene->removeAllCollisionObjects();
        if (success[0] && success[1] && success[2])
        {
            OMPL_INFORM("VALID STATE");
            return true;
        }
        return false;
    }


protected:
private:
    robot_model::RobotModelPtr robot_model;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene;
    collision_detection::AllowedCollisionMatrixPtr acm_;
    moveit_msgs::PlanningScene moveit_scene;
    robot_state::JointModelGroup *planning_group;
    std::shared_ptr<panda_ik> panda_ik_solver;
    Isometry3d base_1st, base_2nd, base_3rd;
    Isometry3d Sobj_grasp[3], Gobj_grasp[3];
    moveit_msgs::CollisionObject stefan_obj;
};
