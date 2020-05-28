#include <triple_constraint_planner/planner/newRRT.h>

#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::newRRT::newRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
    : base::Planner(si, addIntermediateStates ? "newRRTintermediate" : "newRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &newRRT::setRange, &newRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &newRRT::setGoalBias, &newRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &newRRT::setIntermediateStates, &newRRT::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;
    panda_arm = std::make_shared<FrankaModelUpdater>();
}

ompl::geometric::newRRT::~newRRT()
{
    freeMemory();
}

void ompl::geometric::newRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::newRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    maxDistance_ = 9.;
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::newRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::newRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        bool trapped = false;
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(rstate);
            std::cout << "sampling goal" << std::endl;
            si_->printState(rstate);
            
        }

        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        // std::cout << d << " " << maxDistance_ << std::endl;

        // if (d > maxDistance_)
        // {
        //     si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
        //     if (si_->equalStates(nmotion->state, xstate))
        //     {
        //         // trapped = true;
        //         std::cout << "trapped" << std::endl;
        //         dstate = rstate;
        //     }
        //     else
        //     {
        //         dstate = xstate;
        //     }
        //     // std::cout << "d is bigger than maxDistance" << std::endl;
        //     // si_->printState(dstate);
        // }

        if (!trapped)
        {
            if (si_->checkMotion(nmotion->state, dstate))
            {
                std::cout << "check motion module" << std::endl;
                si_->printState(dstate);
                if (addIntermediateStates_)
                {
                    std::vector<base::State *> states;
                    const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

                    if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                        si_->freeState(states[0]);

                    for (std::size_t i = 1; i < states.size(); ++i)
                    {
                        Motion *motion = new Motion;
                        motion->state = states[i];
                        motion->parent = nmotion;
                        nn_->add(motion);

                        nmotion = motion;
                    }
                }
                else
                {
                    Motion *motion = new Motion(si_);
                    si_->copyState(motion->state, dstate);
                    motion->parent = nmotion;
                    nn_->add(motion);

                    nmotion = motion;
                }
                si_->printState(nmotion->state);
                double dist = 0.0;
                bool sat = goal->isSatisfied(nmotion->state, &dist);
                // bool sat = isSatisfied(nmotion->state, &dist);
                if (sat)
                {
                    approxdif = dist;
                    solution = nmotion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = nmotion;
                }
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

// bool ompl::geometric::newRRT::isSatisfied(const ob::State *st, double *distance) const
// {
//     auto *s = st->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
//     Eigen::Matrix<double, 7, 1> joint;
//     for (int i = 0; i < 7; i++)
//         joint[i] = s->values[i];
//     // std::cout << s->values[i] << std::endl;
//     std::cout << joint.transpose() << std::endl;
//     Eigen::Affine3d left_Lgrasp = panda_arm->getTransform(joint);

//     Eigen::Affine3d base_obj = grp.base_serve * left_Lgrasp * grp.Lgrasp_obj;

//     Eigen::Vector3d rpy = base_obj.linear().eulerAngles(0, 1, 2);
//     double roll = rpy[0]; // 90
//     *distance = abs(rad2deg(roll) - 90) + abs(rad2deg(rpy[2]));

//     std::cout << *distance << std::endl;
//     if (*distance < 10 ) 
//     {
//         return true;
//     }

//     return false;
// }



void ompl::geometric::newRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
