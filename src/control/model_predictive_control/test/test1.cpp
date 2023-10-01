#include "model_predictive_control/MPC.hpp"

#include <stdio.h>
#include <iostream>
using namespace task;
using namespace Eigen;

int main(){

    std::cout <<"Prova\n";
    double dT=1.0/400;
    std::string robot_name = "solo12";
    MPC mpc(robot_name, dT);

    Eigen::VectorXd q_init(12);
    q_init << 0.0, 0.3, -0.6, 0.0, 0.3, -0.6, 0.0, -0.3, 0.6, 0.0, -0.3, 0.6;
    Eigen::VectorXd qdot_init(12);
    qdot_init << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    GeneralizedPose gen_pose;
    gen_pose.base_acc = {0, 0, 0};
    gen_pose.base_vel = {0, 0, 0};
    gen_pose.base_pos = {0, 0, 0};

    // Base angular quantities
    gen_pose.base_angvel = {0, 0, 0};
    gen_pose.base_quat = {0, 0, 0, 1};

    // Swing feet linear quantities
    gen_pose.feet_acc = {};
    gen_pose.feet_vel = {};
    gen_pose.feet_pos = {};

    gen_pose.contact_feet_names = {"FL", "FR", "HL", "HR"};

    GeneralizedPoseWithTime gen_pose_time;
    gen_pose_time.gen_pose = gen_pose;
    gen_pose_time.time = 0;

    GeneralizedPosesWithTime gen_poses_time;
    std::vector<GeneralizedPoseWithTime> gen_poses = {gen_pose_time};
    gen_poses_time.generalized_poses_with_time = gen_poses;

    std::cout <<"Prova di stampa\n";

    std::vector<Eigen::VectorXd> sol(1);
    sol = mpc.solve_MPC(q_init, qdot_init, gen_poses_time);

    std::cout <<"Soluzione MPC:\n" <<sol[0];
}