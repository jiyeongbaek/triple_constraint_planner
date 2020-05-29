
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <algorithm>

#include <ompl/base/Constraint.h>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <triple_constraint_planner/kinematics/KinematicChain.h>
#include <triple_constraint_planner/kinematics/panda_model_updater.h>

#include <ctime>

#include <boost/algorithm/string.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
class KinematicChainConstraint : public ompl::base::Constraint
{
public:
    KinematicChainConstraint(unsigned int links) : ompl::base::Constraint(21, 4)
    {
        grasping_point grp;

        qinit_1st = grp.start.segment<7>(0);
        qinit_2nd = grp.start.segment<7>(7);
        qinit_3rd = grp.start.segment<7>(14);

        base_1st = grp.base_1st;
        base_2nd = grp.base_2nd;
        base_3rd = grp.base_3rd;

        offset_R = Eigen::Affine3d::Identity();
        offset_R.linear() = Eigen::AngleAxisd(M_PI / 7, Eigen::Vector3d::UnitX()).toRotationMatrix();

        panda_arm = std::make_shared<FrankaModelUpdater>();
        /* first closed chain */
        init_serve1 = base_1st * panda_arm->getTransform(qinit_1st) * offset_R;
        init_main1= base_3rd * panda_arm->getTransform(qinit_3rd);
        init1 = init_serve1.inverse() * init_main1;

        /* second closed chain */
        init_serve2 = base_2nd * panda_arm->getTransform(qinit_2nd);
        init_main2 = base_3rd * panda_arm->getTransform(qinit_3rd);
        init2 = init_serve2.inverse() * init_main2;

        maxIterations = 250;
    }

    bool project(Eigen::Ref<Eigen::VectorXd> x) const override
    {
        // Newton's method
        unsigned int iter = 0;
        double norm1 = 0;
        double norm2 = 0;
        double norm3 = 0;
        double norm4 = 0;
        Eigen::VectorXd f(getCoDimension());
        Eigen::MatrixXd j(getCoDimension(), n_);
        function(x, f);
        while ( ( (norm1 = f[0] > tolerance1_)  || (norm2 = f[1]) > tolerance2_ || (norm3 = f[2] > tolerance1_)  || (norm4 = f[3]) > tolerance2_ )
                    && iter++ < maxIterations)
        {
            jacobian(x, j);
            x -= 0.30 * j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
            function(x, f);
        }
        
        if ((norm1 < tolerance1_) && (norm2 < tolerance2_) && (norm3 < tolerance1_) && (norm4 < tolerance2_))
        {
            // OMPL_INFORM("ITER : %d  / norm1 : %f  / norm2 : %f  "  , iter, norm1, norm2);
            return true;
        }
        else    
            return false;
        // return (norm1 < squaredTolerance1) && (norm2 < squaredTolerance2);
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        // Eigen::VectorXd &&temp1 = x.segment<7>(0);
        // Eigen::VectorXd &&temp2 = x.segment<7>(7);
        // Eigen::VectorXd &&temp3 = x.segment<7>(14);
        // std::cout << "temp : " << temp1.transpose() << " " << temp2.transpose() << " " << temp3.transpose() << std::endl;

        /* first closed chain */
        Eigen::Affine3d lt1 = base_1st * panda_arm->getTransform(x.segment<7>(0)) * offset_R;
        Eigen::Affine3d rt1 = base_3rd * panda_arm->getTransform(x.segment<7>(14));
        Eigen::Affine3d result1 = lt1.inverse() * rt1;

        double p1 = (result1.translation() - init1.translation()).norm();
        Eigen::Quaterniond cur_q1(result1.linear());
        Eigen::Quaterniond ori_q1(init1.linear());
        double r1 = cur_q1.angularDistance(ori_q1);

        /* second closed chain */
        Eigen::Affine3d lt2 = base_2nd * panda_arm->getTransform(x.segment<7>(7));
        Eigen::Affine3d rt2 = base_3rd * panda_arm->getTransform(x.segment<7>(14));
        Eigen::Affine3d result2 = lt2.inverse() * rt2;

        double p2 = (result2.translation() - init2.translation()).norm();
        Eigen::Quaterniond cur_q2(result2.linear());
        Eigen::Quaterniond ori_q2(init2.linear());
        double r2 = cur_q2.angularDistance(ori_q2);

        out[0] = p1;
        out[1] = r1;
        out[2] = p2;
        out[3] = r2;
       
    }


    void setTolerance(const double tolerance1, const double tolerance2)
    {
        if (tolerance1 <= 0 || tolerance2 <= 0)
            throw ompl::Exception("ompl::base::Constraint::setProjectionTolerance(): "
                                  "tolerance must be positive.");
        tolerance1_ = tolerance1;
        tolerance2_ = tolerance2;
        OMPL_INFORM("Set tolerance to %f and %f", tolerance1_, tolerance2_);
    }

    bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
    {
        Eigen::VectorXd f(getCoDimension());
        function(x, f);
        return f.allFinite() && f[0] <= tolerance1_ && f[1] <= tolerance2_ && f[2] <= tolerance1_ && f[3] <= tolerance2_;
    }

protected:
    double tolerance1_, tolerance2_;

private:
    Eigen::Matrix<double, 7, 1> q_1st, q_2nd, q_3rd;
    Eigen::Matrix<double, 7, 1> qinit_1st, qinit_2nd, qinit_3rd;
    Eigen::Affine3d init_serve1, init_main1, init1, init_serve2, init_main2, init2;

    std::shared_ptr<FrankaModelUpdater> panda_arm;
    Eigen::Matrix<double, 4, 4> init_;

    int maxIterations;

    Eigen::Affine3d base_1st, base_2nd, base_3rd;
    Eigen::Affine3d offset_R;
};

typedef std::shared_ptr<KinematicChainConstraint> ChainConstraintPtr;

// class KinematicChainConstraint : public Constraint_new
// {
// public:
//     /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
//     KinematicChainConstraint(unsigned int links, Eigen::VectorXd start) : Constraint_new(links, 2)
//     {
//         for (int i = 0; i < 7; i++)
//         {
//             qinit_serve[i] = start[i];
//             qinit_main[i] = start[i + 7];
//         }
//         base_1st.translation() = Eigen::Vector3d(0.0, 0.2, 0.0);
//         base_3rd.translation() = Eigen::Vector3d(0.0, -0.2, 0.0);

//         panda_model = std::make_shared<FrankaModelUpdater>(q_1st);
//         panda_model = std::make_shared<FrankaModelUpdater>(q_3rd);

//         init_serve = base_1st * panda_model->getTransform(qinit_serve);
//         init_main = base_3rd * panda_model->getTransform(qinit_main);
//         init = init_serve.inverse() * init_main;
//         init_tr = init.linear().transpose();
//     }

//     /*actual constraint function, state "x" from the ambient space */
//     void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
//     {

//         Eigen::VectorXd &&temp = x.segment(0, 7);
//         Eigen::VectorXd &&temp2 = x.segment(7, 7);

//         Eigen::Affine3d lt = base_1st * panda_model->getTransform(temp);
//         Eigen::Affine3d rt = base_3rd * panda_model->getTransform(temp2);

//         Eigen::Affine3d result = lt.inverse() * rt;

//         double r;
//         Eigen::Quaterniond cur_q(result.linear());
//         Eigen::Quaterniond ori_q(init.linear());
//         r = cur_q.angularDistance(ori_q);

//         double d;
//         d = (result.translation() - init.translation()).norm();

//         // double d_rot;
//         // d_rot = (  init_tr * result.linear() ).log().norm();

//         out[0] = d;
//         out[1] = r;
//     }

//     /* this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
//     void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
//     {
//         double h = 1e-4;
//         double d, d_rot;
//         double r;
//         Eigen::Quaterniond ori_q(init.linear());
//         for (int i = 0; i < 14; i++)
//         {
//             Eigen::VectorXd &&q_temp = x;
//             q_temp(i) = x(i) + h;
//             Eigen::Affine3d lt_1 = base_1st * panda_model->getTransform(q_temp.segment(0, 7));
//             Eigen::Affine3d rt_1 = base_3rd * panda_model->getTransform(q_temp.segment(7, 7));
//             Eigen::Affine3d result_1 = lt_1.inverse() * rt_1;

//             // d_rot = ( init_tr * result_1.linear() ).log().norm();

//             Eigen::Quaterniond cur1_q(result_1.linear());
//             r = cur1_q.angularDistance(ori_q);
//             d = (result_1.translation() - init.translation()).norm();
//             double g1_d = d;
//             double g1_r = r;

//             q_temp(i) = x(i) - h;
//             Eigen::Affine3d lt_2 = base_1st * panda_model->getTransform(q_temp.segment(0, 7));
//             Eigen::Affine3d rt_2 = base_3rd * panda_model->getTransform(q_temp.segment(7, 7));
//             Eigen::Affine3d result_2 = lt_2.inverse() * rt_2;
//             Eigen::Quaterniond cur2_q(result_2.linear());
//             r = cur2_q.angularDistance(ori_q);
//             d = (result_2.translation() - init.translation()).norm();
//             // d_rot = ( init_tr * result_2.linear() ).log().norm();

//             double g2_d = d;
//             double g2_r = r;
//             out(0, i) = (g1_d - g2_d) / (2 * h);
//             out(1, i) = (g1_r - g2_r) / (2 * h);
//         }
//     }

// private:
//     Eigen::Matrix<double, 7, 1> q_1st, q_3rd;
//     Eigen::Matrix<double, 7, 1> qinit_serve, qinit_main;
//     Eigen::Affine3d init_serve, init_main, init, init_tr;

//     std::shared_ptr<FrankaModelUpdater> init_panda_model, panda_model;
//     std::shared_ptr<FrankaModelUpdater> panda_model, init_panda_model;
//     Eigen::Matrix<double, 4, 4> init_;
// };