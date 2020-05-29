#pragma once
#include <utility>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/base/spaces/constraint/AtlasChart.h"
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "ompl/base/spaces/constraint/TangentBundleStateSpace.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <triple_constraint_planner/kinematics/KinematicChain.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
namespace ob = ompl::base;
class jy_ConstrainedValidStateSampler : public ob::ConstrainedValidStateSampler
{
public:
    jy_ConstrainedValidStateSampler(const ob::SpaceInformation *si) : ob::ConstrainedValidStateSampler(si), 
                                                                    sampler_(si->getStateSpace()->allocStateSampler()),
                                                                    constraint_(si->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getConstraint()),
                                                                    obj_space_(),
                                                                    obj_sampler_(obj_space_->allocStateSampler())
    // : ValidStateSampler(si), sampler_(si->getStateSpace()->allocStateSampler()), constraint_(si->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getConstraint())
    {
        std::cout << "setting to default sampler" << std::endl;
        lower_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        upper_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    }

    bool sample(ob::State *state) override
    {
        unsigned int tries = 0;
        bool valid;
        do
        {
            ob::State *obj_state = obj_space_->allocState();
            obj_sampler_->sampleUniform(obj_state);
            sampler_->sampleUniform(state);
            // bool testbounds = testBounds(state);
            // bool validity = si_->isValid(state);
            // bool constraint = constraint_->isSatisfied(state);
            // si_->printState(state);
            // std::cout << testbounds << " " << validity << " " << constraint << std::endl;
            
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

private:
    ob::StateSamplerPtr sampler_, obj_sampler_;
    std::shared_ptr<ob::SE3StateSpace> obj_space_;
    const ob::ConstraintPtr constraint_;
    Eigen::Matrix<double, 14, 1> lower_limit, upper_limit;
};

typedef std::shared_ptr<jy_ConstrainedValidStateSampler> jy_ConstrainedValidStateSamplerPtr;
