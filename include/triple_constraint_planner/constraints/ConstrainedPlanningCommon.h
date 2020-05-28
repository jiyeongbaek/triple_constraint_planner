#pragma once

#include <iostream>
#include <fstream>

#include <boost/format.hpp>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <triple_constraint_planner/constraints/ConstraintFunction.h>
#include <triple_constraint_planner/base/jy_ConstrainedValidStateSampler.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
// #include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <triple_constraint_planner/base/jy_ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <triple_constraint_planner/planner/newRRT.h>
#include <triple_constraint_planner/planner/newPRM.h>
#include <triple_constraint_planner/planner/newRRTConnect.h>
#include <ompl/tools/benchmark/Benchmark.h>
// #include <ompl/base/goals/GoalLazySamples.h>
#include <triple_constraint_planner/base/jy_GoalLazySamples.h>

#include <ompl/base/spaces/SE3StateSpace.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::magic;
namespace ot = ompl::tools;
using namespace Eigen;
using namespace std;
enum PLANNER_TYPE
{
    RRT,
    RRTConnect,
    PRM,
    newRRT,
    newPRM,
    newRRTConnect
};

std::istream &operator>>(std::istream &in, enum PLANNER_TYPE &type)
{
    std::string token;
    in >> token;
    if (token == "RRT")
        type = RRT;
    else if (token == "RRTConnect")
        type = RRTConnect;
    else if (token == "PRM")
        type = PRM;
    else if (token == "newRRT")
        type = newRRT;
    else if (token == "newPRM")
        type = newPRM;
    else if (token == "newRRTConnect")
        type = newRRTConnect;
    else
        in.setstate(std::ios_base::failbit);

    return in;
}

struct ConstrainedOptions
{
    double delta;
    double lambda;
    double tolerance1;
    double tolerance2;
    double time;
    unsigned int tries;
    double range;
};

class ConstrainedProblem
{
public:
    ConstrainedProblem(ob::StateSpacePtr space_, ChainConstraintPtr constraint_)
        : space(std::move(space_)), constraint(std::move(constraint_))
    {
        OMPL_INFORM("Using Projection-Based State Space!");
        css = std::make_shared<jy_ProjectedStateSpace>(space, constraint);
        csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
        css->setup();
        ss = std::make_shared<og::SimpleSetup>(csi);

        csi->setValidStateSamplerAllocator([](const ob::SpaceInformation *si) -> std::shared_ptr<ob::ValidStateSampler> {
            return std::make_shared<jy_ConstrainedValidStateSampler>(si);
        });

        base_1st = grp.base_1st;
        base_2nd = grp.base_2nd;
        base_3rd = grp.base_3rd;
        obj_grasp1 = grp.obj_grasp1;
        obj_grasp2 = grp.obj_grasp2;
        obj_grasp3 = grp.obj_grasp3;
    }

    /* . The distance between each point in the discrete geodesic is tuned by the "delta" parameter
         Valid step size for manifold traversal with delta*/
    void setConstrainedOptions()
    {
        // rrt 되는거
        c_opt.delta = 0.3; //0.075

        c_opt.lambda = 5.0;
        c_opt.tolerance1 = 0.002; //0.001
        c_opt.tolerance2 = 0.025; // 1degree
        c_opt.time = 60.;
        c_opt.tries = 200;
        // c_opt.range = 1.5;

        constraint->setTolerance(c_opt.tolerance1, c_opt.tolerance2);
        constraint->setMaxIterations(c_opt.tries);

        css->setDelta(c_opt.delta);
        css->setLambda(c_opt.lambda);
    }

    void setStartAndGoalStates()
    {
        ob::ScopedState<> sstart(css);
        sstart->as<ob::ConstrainedStateSpace::StateType>()->copy(grp.start);
        ss->setStartState(sstart);
    }

    ob::PlannerStatus solveOnce(bool goalsampling, const std::string &name = "projection")
    {
        ss->setup();
        ob::jy_GoalSamplingFn samplingFunction = [&](const ob::jy_GoalLazySamples *gls, ob::State *result) {
            return sampleIKgoal(gls, result);
        };
        std::shared_ptr<ompl::base::jy_GoalLazySamples> goal;

        if (goalsampling)
        {
            goal = std::make_shared<ompl::base::jy_GoalLazySamples>(ss->getSpaceInformation(), samplingFunction, false);
            ob::State *first_goal = csi->allocState();
            if (sampleIKgoal(first_goal))
            {
                csi->printState(first_goal);
                goal->addState(first_goal);
            }
            
            // goal->startSampling();
            ss->setGoal(goal);
        }

        ob::PlannerStatus stat = ss->solve(c_opt.time);
        dumpGraph("test");
        if (stat)
        {
            ompl::geometric::PathGeometric path = ss->getSolutionPath();
            // if (!path.check())
            //     OMPL_WARN("Path fails check!");
            if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
                OMPL_WARN("Solution is approximate.");
            path.printAsMatrix(std::cout);
            OMPL_INFORM("Interpolating path & Dumping path to `%s_path.txt`.", name.c_str());
            path.interpolate();
            std::ofstream pathfile("/home/jiyeong/catkin_ws/" + name + "_path.txt");
            path.printAsMatrix(pathfile);
            pathfile.close();
            std::cout << std::endl;
        }
        else
            OMPL_WARN("No solution found.");

        if (goalsampling)
            goal->as<ob::jy_GoalLazySamples>()->stopSampling();

        return stat;
    }

    void dumpGraph(const std::string &name)
    {
        ob::PlannerData data(csi);
        pp->getPlannerData(data);

        std::ofstream graphfile("/home/jiyeong/catkin_ws/" + name + "_path.graphml");
        data.printGraphML(graphfile);
        graphfile.close();

        std::ofstream graphfile2("/home/jiyeong/catkin_ws/" + name + "_path.dot");
        data.printGraphviz(graphfile2);
        graphfile2.close();

        OMPL_INFORM("Dumping planner graph to `%s_graph.graphml`.", name.c_str());
    }

    bool sampleIKgoal(const ob::jy_GoalLazySamples *gls, ob::State *result)
    {
        int stefan_tries = 500;
        std::shared_ptr<panda_ik> panda_ik_solver = std::make_shared<panda_ik>();
        Affine3d base_obj = grp.base_obj;
        
        while (--stefan_tries)
        {
            Affine3d target_1 = base_1st.inverse() * base_obj * obj_grasp1;
            Affine3d target_2 = base_2nd.inverse() * base_obj * obj_grasp2;
            Affine3d target_3 = base_3rd.inverse() * base_obj * obj_grasp3;

            Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();
            int tries = 50;
            while (--tries)
            {
                bool success1, success2, success3;
                success1 = panda_ik_solver->randomSolve(target_1, sol.segment<7>(0));
                success2 = panda_ik_solver->randomSolve(target_2, sol.segment<7>(7));
                success3 = panda_ik_solver->randomSolve(target_3, sol.segment<7>(14));
                if (success1 && success2 && success3)
                {
                    if (gls->getSpaceInformation()->isValid(result))
                    {
                        return true;
                    }
                }
                else
                    break;
            }
        }
        return false;
    }

    bool sampleIKgoal(ob::State *result)
    {
        std::shared_ptr<panda_ik> panda_ik_solver = std::make_shared<panda_ik>();
        Affine3d base_obj = grp.base_obj;
        
        Affine3d target_1 = base_1st.inverse() * base_obj * obj_grasp1;
        Affine3d target_2 = base_2nd.inverse() * base_obj * obj_grasp2;
        Affine3d target_3 = base_3rd.inverse() * base_obj * obj_grasp3;

        Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();
        bool success1, success2, success3;
        success1 = panda_ik_solver->solve(grp.start.segment<7>(0), target_1, sol.segment<7>(0));
        success2 = panda_ik_solver->solve(grp.start.segment<7>(7), target_2, sol.segment<7>(7));
        success3 = panda_ik_solver->solve(grp.start.segment<7>(14), target_3, sol.segment<7>(14));
        
        if (success1 && success2 && success3)
        {
            if (csi->isValid(result))
            {
                return true;
            }
        }

        return false;
    }
    template <typename _T>
    std::shared_ptr<_T> createPlanner()
    {
        auto &&planner = std::make_shared<_T>(csi);
        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerIntermediate()
    {
        auto &&planner = std::make_shared<_T>(csi, true);
        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerRange()
    {
        auto &&planner = createPlanner<_T>();

        planner->setRange(c_opt.range);

        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerRange(bool intermediate)
    {
        auto &&planner = createPlannerIntermediate<_T>();

        planner->setRange(c_opt.range);

        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerRangeProj(const std::string &projection)
    {
        const bool isProj = projection != "";
        auto &&planner = createPlannerRange<_T>();

        if (isProj)
            planner->setProjectionEvaluator(projection);

        return std::move(planner);
    }

    ob::PlannerPtr getPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        ob::PlannerPtr p;
        switch (planner)
        {
        case RRT:
            p = createPlannerRange<og::RRT>();
            break;
        case RRTConnect:
            p = createPlannerRange<og::RRTConnect>();
            break;

        case PRM:
            p = createPlanner<og::PRM>();
            break;

        case newRRT:
            p = createPlanner<og::newRRT>();
            break;

        case newPRM:
            p = createPlanner<og::newPRM>();
            break;
        case newRRTConnect:
            p = createPlannerRange<og::newRRTConnect>();
            break;
        }
        return p;
    }

    void setPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        pp = getPlanner(planner, projection);
        ss->setPlanner(pp);
    }

    ob::StateSpacePtr space;
    ChainConstraintPtr constraint;

    ob::ConstrainedStateSpacePtr css;
    // std::shared_ptr<jy_ConstrainedSpaceInformation> csi;
    ob::ConstrainedSpaceInformationPtr csi;
    ob::PlannerPtr pp;
    og::SimpleSetupPtr ss;

    struct ConstrainedOptions c_opt;
    Affine3d obj_grasp1, obj_grasp2, obj_grasp3, base_1st, base_2nd, base_3rd;
    grasping_point grp;

protected:
    ompl::RNG rng_;
};