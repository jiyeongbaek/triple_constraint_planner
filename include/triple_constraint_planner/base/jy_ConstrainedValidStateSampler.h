#pragma once
#include <utility>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <triple_constraint_planner/kinematics/KinematicChain.h>
#include <triple_constraint_planner/kinematics/panda_model_updater.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
namespace ob = ompl::base;
class jy_ConstrainedValidStateSampler : public ob::ConstrainedValidStateSampler
{
public:
    jy_ConstrainedValidStateSampler(const ob::SpaceInformation *si) : ob::ConstrainedValidStateSampler(si), 
                                                                    sampler_(si->getStateSpace()->allocStateSampler()),
                                                                    constraint_(si->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getConstraint())                                                                   
    // : ValidStateSampler(si), sampler_(si->getStateSpace()->allocStateSampler()), constraint_(si->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getConstraint())
    {
        lower_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        upper_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

        base_1st = grp.base_1st;
        base_2nd = grp.base_2nd;
        base_3rd = grp.base_3rd;
        obj_grasp1 = grp.obj_grasp1;
        obj_grasp2 = grp.obj_grasp2;
        obj_grasp3 = grp.obj_grasp3;
        grasp3_obj = grp.grasp3_obj;

        obj_space_ = std::make_shared<ob::SE3StateSpace>();
        ob::RealVectorBounds bounds(3);
        // bounds.setLow(0, 0.5);
        // bounds.setHigh(0, 1.4);
        // bounds.setLow(1, -1.0);
        // bounds.setHigh(1, 1.0);
        // bounds.setLow(2, 0.8);
        // bounds.setHigh(2, 1.05);
        
        bounds.setLow(0, 0.6);
        bounds.setHigh(0, 1.4);
        bounds.setLow(1, -0.5);
        bounds.setHigh(1, 0.5);
        bounds.setLow(2, 0.8);
        bounds.setHigh(2, 1.05);
        obj_space_->setBounds(bounds);
        obj_sampler_ =obj_space_->allocStateSampler();

        panda_ik_solver = std::make_shared<panda_ik>();        
    }

    bool sample(ob::State *state) override
    {
        unsigned int tries = 0;
        bool valid;
        do
        {
            ob::State *obj_state = obj_space_->allocState();
            obj_sampler_->sampleUniform(obj_state);
            sampleIKgoal(obj_state, state);
        }
        while ( !(valid = testBounds(state) && constraint_->isSatisfied(state) && si_->isValid(state) ) && ++tries < 10);
        if (valid)
        {
            OMPL_INFORM("ADD VALID STATE");
            return true;
        }
        else
            OMPL_WARN("INVALID STATE");
        // std::cout << valid << std::endl;
        // return valid; 
    }

    bool testBounds(ob::State *state) const
    {
        auto &&rstate = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
        for (unsigned int i = 0; i < 14; ++i)
        {
            if (rstate->values[i] > upper_limit[i] || rstate->values[i] < lower_limit[i])
                return false;
        }
        return true;
    }

    bool sampleIKgoal(ob::State *obj_state, ob::State *result)
    {
        auto *se3state = obj_state->as<ob::SE3StateSpace::StateType>();
        ob::State *ik_result = si_->allocState();
        Affine3d base_obj;
        base_obj.translation() = Eigen::Vector3d(se3state->getX(), se3state->getY(), se3state->getZ());
        // base_obj.linear() = Eigen::Quaterniond(se3state->rotation().w, 
        //                                         se3state->rotation().x, 
        //                                         se3state->rotation().y, 
        //                                         se3state->rotation().z).toRotationMatrix();
        base_obj.linear().setIdentity();
        Affine3d target_1 = base_1st.inverse() * base_obj * obj_grasp1;
        Affine3d target_2 = base_2nd.inverse() * base_obj * obj_grasp2;
        Affine3d target_3 = base_3rd.inverse() * base_obj * obj_grasp3;
       
        Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();

        // sol.segment<7>(14)= panda_ik_solver->getRandomConfig();
        // Affine3d panda3_grasp3 = panda_ik_solver->fk(sol.segment<7>(14));
        // if (panda3_grasp3.translation()[2] < 0.1)
        //     return false;
        // Affine3d base_obj = base_3rd * panda3_grasp3 * grasp3_obj;
        // Affine3d target_1 = base_1st.inverse() * base_obj * obj_grasp1;

        // if (target_1.translation()[0] > 1.3)
        //     return false;

        // Affine3d target_2 = base_2nd.inverse() * base_obj * obj_grasp2;
        
        if (panda_ik_solver->randomSolve(target_1, sol.segment<7>(0)) && 
            panda_ik_solver->randomSolve(target_2, sol.segment<7>(7)) && 
            panda_ik_solver->randomSolve(target_3, sol.segment<7>(14)) )
        {
            return true;
        }
        sol.setZero();

        return false;
    }

private:
    ob::StateSamplerPtr sampler_, obj_sampler_;
    std::shared_ptr<ob::SE3StateSpace> obj_space_;
    const ob::ConstraintPtr constraint_;
    Eigen::Matrix<double, 14, 1> lower_limit, upper_limit;
    grasping_point grp;
    Affine3d obj_grasp1, obj_grasp2, obj_grasp3, base_1st, base_2nd, base_3rd, grasp3_obj;
    std::shared_ptr<panda_ik> panda_ik_solver;
};

typedef std::shared_ptr<jy_ConstrainedValidStateSampler> jy_ConstrainedValidStateSamplerPtr;
