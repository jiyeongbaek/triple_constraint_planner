#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
class grasping_point
{
public:
    grasping_point()
    {
        Vector3d z_offset(0, 0, -0.103);

        base_1st = Eigen::Affine3d::Identity();
        base_2nd = Eigen::Affine3d::Identity();
        base_3rd = Eigen::Affine3d::Identity();

        base_1st.translation() = Eigen::Vector3d(0, 0.3, 0.6);
        base_2nd.translation() = Eigen::Vector3d(0, -0.3, 0.6);
        base_3rd.translation() = Eigen::Vector3d(1.6, 0.0, 0.6);
        base_3rd.linear() << -1, 0, 0,
            0, -1, 0,
            0, 0, 1;

        bool rotate_chair = true;
        if (rotate_chair)
        {
            planning_group = "panda_arms";
            hand_group = "panda_hands";

            obj_grasp1.linear() = Quaterniond(0.497631, 0.483366, 0.516627, 0.501814).toRotationMatrix();
            obj_grasp1.translation() = Vector3d(-0.4258375, 0.21016605, 0.0207994) + obj_grasp1.linear() * z_offset;

            obj_grasp2.linear() = Quaterniond(-0.0373972, 0.996136, 0.079409, 0.00298125).toRotationMatrix();
            obj_grasp2.translation() = Vector3d(-0.7898177, -0.06157755, 0.3247905) + obj_grasp2.linear() * z_offset;

            obj_grasp3.linear() = Quaterniond(0.507219, 0.492676, -0.507217, -0.492677).toRotationMatrix();
            obj_grasp3.translation() = Vector3d(-0.128369, 0.20230575, 0.017412) + obj_grasp3.linear() * z_offset;

            grasp1_obj = obj_grasp1.inverse();
            grasp2_obj = obj_grasp2.inverse();
            grasp3_obj = obj_grasp3.inverse();

            base_obj.linear() = AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            base_obj.translation() = Vector3d(1.15, 0.05, 0.85);

            start << 0.8538476617343256, 0.7765987891970887, -1.38718092553011, -1.9162352330353676, 2.693557656819878, 2.209230516957901, -2.8518449420397336,
                0.0402869216512108, -0.23189200532557086, 0.20321293759873837, -1.2398464683701635, 0.13884452366534228, 1.0347525181748924, 0.9375305860020455,
                2.4288973744080127, 0.2356190832102002, -2.6487764272706724, -2.409884568379378, 2.7754012268293335, 2.451555244441547, 2.786489214331766;

            goal << 0.437031, 0.47465, -0.994905, -1.75628, 2.50555, 2.32889, -1.45654, 0.438398, 0.633263, -0.943085, -2.42709, 1.29725, 0.927702, -0.817462, 2.18516, 0.42681, -2.28174, -2.38358, 2.89136, 2.55816, 1.34693;
        }

        else
        {
            std::cout << "***CHAIR UP MODE" << std::endl;
            planning_group = "panda_chair_up";
            hand_group = "hand_chair_up";

            obj_grasp2.linear() = Quaterniond(-0.0540426, -0.0582587, 0.67793, 0.730818).toRotationMatrix();
            obj_grasp2.translation() = Vector3d(-0.6601151, -0.02137197, 0.02044866) + obj_grasp2.linear() * z_offset;

            obj_grasp3.linear() = Quaterniond(0.0102825, 0.707032, -0.707032, 0.0102825).toRotationMatrix();
            obj_grasp3.translation() = Vector3d(-0.133369, 0.26831887, 0.01643235) + obj_grasp3.linear() * z_offset;

            grasp2_obj = obj_grasp2.inverse();
            grasp3_obj = obj_grasp3.inverse();

            // start << 1.7635811732933235, -1.4411345207422865, -1.964651184080014, -1.7905553615439762, 0.20378384311742412, 1.7390337027885823, -2.800300667744541, -2.507227794231461, -0.23624109784362163, 2.5633123073239905, -2.268388140289912, 0.24936065684482742, 2.4538909693928335, -0.9104041928398361;
            // goal <<  -1.3467140413650676, 1.759152969838163, 1.7421234462009596, -2.29020266856861, -0.16968203574810806, 2.3272818650833904, -2.090762525897875, 0.4358071209224652, 0.20526492757492862, -0.8031983820172117, -1.6474676957518197, 0.17823531304271836, 1.7776013922868934, -1.1806954807587389;
        }
    }

    Affine3d base_1st, base_2nd, base_3rd, base_obj;
    Affine3d obj_grasp1, obj_grasp2, obj_grasp3, grasp3_obj, grasp2_obj, grasp1_obj;
    Matrix<double, 21, 1> start, goal;
    std::string planning_group, hand_group;
};