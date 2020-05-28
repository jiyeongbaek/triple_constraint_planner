
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
         for (int i = 0; i < 7; i++)
        {
            qinit_1st[i] = grp.start[i];
            qinit_2nd[i] = grp.start[i + 7];
            qinit_3rd[i] = grp.start[i + 14];
        }

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
        init_serve2 = base_2nd * panda_arm->getTransform(qinit_2nd) * offset_R;
        init_main2 = base_3rd * panda_arm->getTransform(qinit_3rd);
        init2 = init_serve2.inverse() * init_main2;

        maxIterations = 150;
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
            x -= 0.20 * j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
            OMPL_INFORM("ITER : %d  / norm1 : %f  / norm2 : %f  / norm1 : %f  / norm2 : %f  ", iter, f[0], f[1], f[2], f[3]);
            function(x, f);
        }
        // cout << "F : " << f.transpose() << endl;
        // cout << "f tail : " << f.tail(3).transpose() << endl;
        // cout << f.tail(3).squaredNorm() << endl;
        // cout << "x          : " << x.transpose() << endl;
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
        Eigen::VectorXd &&temp1 = x.segment(0, 7);
        Eigen::VectorXd &&temp2 = x.segment(7, 7);
        Eigen::VectorXd &&temp3 = x.segment(14, 7);

        /* first closed chain */
        Eigen::Affine3d lt1 = base_1st * panda_arm->getTransform(temp1) * offset_R;
        Eigen::Affine3d rt1 = base_3rd * panda_arm->getTransform(temp3);
        Eigen::Affine3d result1 = lt1.inverse() * rt1;

        double p1 = (result1.translation() - init1.translation()).norm();
        Eigen::Quaterniond cur_q1(result1.linear());
        Eigen::Quaterniond ori_q1(init1.linear());
        double r1 = cur_q1.angularDistance(ori_q1);

        /* second closed chain */
        Eigen::Affine3d lt2 = base_2nd * panda_arm->getTransform(temp2) * offset_R;
        Eigen::Affine3d rt2 = base_3rd * panda_arm->getTransform(temp3);
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

    /* this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
    // void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    // {
    //     Eigen::VectorXd &&q_temp = x;
    //     Eigen::Affine3d lt = base_1st * panda_arm->getTransform(q_temp.segment(0, 7)) * offset_R;
    //     Eigen::Affine3d rt = base_3rd * panda_arm->getTransform(q_temp.segment(7, 7));
    //     Eigen::Affine3d result = lt.inverse() * rt;
    //     Eigen::Matrix3d r_diff = init.linear().transpose() * result.linear();

    //     Eigen::Vector3d rpy = result.linear().eulerAngles(0, 1, 2);
    //     // rpy(0) = atan2(result(2, 1), result(2, 2));
    //     // rpy(1) = -asin(result(2, 0));
    //     // rpy(2) = atan2(result(1, 0), result(0, 0));
        
    //     // E_rpy(0,0) = cos(rpy(2))/cos(rpy(1));
    //     // E_rpy(0,1) = sin(rpy(2))/cos(rpy(1));
    //     // E_rpy(0,2) = 0.0;

    //     // E_rpy(1,0) = sin(rpy(2));
    //     // E_rpy(1,1) = cos(rpy(2));
    //     // E_rpy(1,2) = 0.0;

    //     // E_rpy(2,0) = -cos(rpy(2))*sin(rpy(1))/cos(rpy(1));
    //     // E_rpy(2,1) = sin(rpy(2))*sin(rpy(1))/cos(rpy(1));
    //     // E_rpy(2,2) = 1.0;

    //     Eigen::MatrixXd offset_R2 = Eigen::MatrixXd::Zero(6, 6);
    //     offset_R2.block<3, 3>(0, 0) = offset_R.linear();
    //     offset_R2.block<3, 3>(3, 3) = offset_R.linear();
        
    //     Eigen::MatrixXd jaco_l_temp = offset_R2 * panda_arm->getJacobian(q_temp.segment(0, 7));
    //     Eigen::MatrixXd jaco_r_temp = panda_arm->getJacobian(q_temp.segment(7, 7));
    //     Eigen::Matrix<double, 6, 7> jaco_l, jaco_r;
    //     jaco_l.topRows(3) = jaco_l_temp.bottomRows(3);
    //     jaco_l.bottomRows(3) = jaco_l_temp.topRows(3);
        
    //     jaco_r.topRows(3) = jaco_r_temp.bottomRows(3);
    //     jaco_r.bottomRows(3) = jaco_r_temp.topRows(3);
        
    //     Eigen::MatrixXd omega_1st = Eigen::MatrixXd::Zero(6, 6);
    //     Eigen::MatrixXd omega_3rd = Eigen::MatrixXd::Zero(6, 6);
    //     omega_1st.block<3, 3>(0, 0) = lt.linear().inverse();
    //     omega_1st.block<3, 3>(3, 3) = lt.linear().inverse();
    //     omega_3rd.block<3, 3>(0, 0) = result.linear() * (rt.linear().inverse());
    //     omega_3rd.block<3, 3>(3, 3) = result.linear() * (rt.linear().inverse());
    //     // std::cout << result.linear() << std::endl;
        
    //     Eigen::MatrixXd wrench = Eigen::MatrixXd::Zero(6, 6);
    //     wrench.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    //     wrench.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    //     wrench.block<3, 3>(0, 3) = -skew_symmetric(result.translation());
        
        
    //     std::cout << "wrench" << std::endl;
    //     std::cout << wrench << std::endl;
    //     Eigen::MatrixXd jaco(6, 14);
    //     jaco.block<6, 7>(0, 0) = -wrench * omega_1st * jaco_l;
    //     jaco.block<6, 7>(0, 7) = omega_3rd * jaco_r;

    //     Eigen::MatrixXd e_matrix = Eigen::MatrixXd::Zero(6, 6);
    //     e_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    //     Eigen::Matrix3d e_R;
    //     e_R << 1, sin(rpy[0]) * sin(rpy[1]) / cos(rpy[1]), -cos(rpy[0])*sin(rpy[1])/cos(rpy[1]), 
    //             0, cos(rpy[0]), sin(rpy[0]),
    //             0, -sin(rpy[0]) / cos(rpy[1]) , cos(rpy[0]) / cos(rpy[1]);
    //     e_matrix.block<3, 3>(3, 3) = e_R ;

    //     // out = e_matrix * jaco;
    //     std::cout << result.linear() << std::endl;
    //     std::cout << std::endl;
    //     out = jaco;
        
    // }

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

    Eigen::Matrix3d skew_symmetric(Eigen::Vector3d x) const
    {
        Eigen::Matrix3d result;
        result.setZero();
        result(0, 1) = -x[2];
        result(0, 2) = x[1];
        result(1, 0) = x[2];
        result(1, 2) = -x[0];
        result(2, 0) = -x[1];
        result(2, 1) = x[0];
        return result;
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