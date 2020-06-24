#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <Eigen/Dense>
#include <cmath>

#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>

#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <triple_constraint_planner/planner/rgpRRTConnect.h>
#include <triple_constraint_planner/base/jy_RegraspValidityCheck.h>
#include <ros/package.h>
using namespace std;
using namespace Eigen;

namespace ob = ompl::base;
namespace og = ompl::geometric;

class planning_setup
{
public:
    planning_setup()
    {
        space = std::make_shared<ob::SE3StateSpace>();
        si = std::make_shared<ob::SpaceInformation>(space);
        pdef = std::make_shared<ob::ProblemDefinition>(si);
        ss = std::make_shared<og::SimpleSetup>(space);
        // vc = std::make_shared<ValidityCheck>(nh, ss->getSpaceInformation());

        // vc = std::make_shared<ValidityCheck>(nh, si);
    }

    void setStartAndGoal()
    {
        ob::ScopedState<> start(space);
        auto *se3state = start->as<ob::SE3StateSpace::StateType>();
        auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        pos->values[0] = 1.20;
        pos->values[1] = -0.3;
        pos->values[2] = 0.751;

        rot->x = 0.0;
        rot->y = 0.0;
        rot->z = 0.0;
        rot->w = 1.0;

        ob::ScopedState<> goal(space);
        auto *se3state2 = goal->as<ob::SE3StateSpace::StateType>();
        auto *pos2 = se3state2->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot2 = se3state2->as<ob::SO3StateSpace::StateType>(1);
        pos2->values[0] = 1.15;
        pos2->values[1] = 0.2;
        pos2->values[2] = 0.85;
        Eigen::Quaterniond q2(AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                              AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                              AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix());
        rot2->x = q2.coeffs().x();
        rot2->y = q2.coeffs().y();
        rot2->z = q2.coeffs().z();
        rot2->w = q2.coeffs().w();
        pdef->setStartAndGoalStates(start, goal);
    }

    bool plan()
    {
        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, 1.1);
        bounds.setHigh(0, 1.4);
        bounds.setLow(1, -0.4);
        bounds.setHigh(1, 0.4);
        bounds.setLow(2, 0.75);
        bounds.setHigh(2, 1.15);
        space->setBounds(bounds);
        std::cout << "  - min: ";
        for (unsigned int i = 0; i < 3; ++i)
            std::cout << bounds.low[i] << " ";
        std::cout << std::endl;
        std::cout << "  - max: ";
        for (unsigned int i = 0; i < 3; ++i)
            std::cout << bounds.high[i] << "  ";
        std::cout << std::endl;

        // si->setStateValidityChecker(vc);

        setStartAndGoal();
        // auto planner(std::make_shared<og::rgpRRTConnect>(si));
        planner = std::make_shared<og::rgpRRTConnect>(si);

        // planner->setRange(0.25); //0.25 - 7, 0.1-24,  0.5-8
        planner->setProblemDefinition(pdef);
        // si->printSettings(std::cout);  // print the settings for this space
        // pdef->print(std::cout);  // print the problem settings

        ob::PlannerStatus solved = planner->ob::Planner::solve(40.0);

        if (solved)
        {
            dumpGraph("test");
    
            auto path = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;

            auto path2 = path->as<og::PathGeometric>();
            path2->printAsMatrix(std::cout);

            std::ofstream result_file(ros::package::getPath("triple_constraint_planner") + "/object_state.txt");
            path2->printAsMatrix(result_file);
            return true;
        }
        else
            std::cout << "No solution found" << std::endl;
        return false;
    }

    void dumpGraph(const std::string &name)
    {
        ob::PlannerData data(si);
        planner->getPlannerData(data);

        std::ofstream graphfile("/home/jiyeong/catkin_ws/" + name + "_path.graphml");
        data.printGraphML(graphfile);
        graphfile.close();

        std::ofstream graphfile2("/home/jiyeong/catkin_ws/" + name + "_path.dot");
        data.printGraphviz(graphfile2);
        graphfile2.close();

        OMPL_INFORM("Dumping planner graph to `%s_graph.graphml`.", name.c_str());
    }


    ob::SpaceInformationPtr si;
    // ob::StateSpaceptr space;
    std::shared_ptr<ob::SE3StateSpace> space;
    // boost::shared_ptr<ob::SE3StateSpace> space;
    ob::ProblemDefinitionPtr pdef;
    std::shared_ptr<RgpValidityChecker> vc;

    og::SimpleSetupPtr ss;
    ob::PlannerPtr planner;
};