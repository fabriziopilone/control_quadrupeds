#include "model_predictive_control/MPC.hpp"

#include <stdio.h>
#include <iostream>
using namespace task;
using namespace Eigen;

int main(){

    std::cout <<"Prova\n";
    double dT=1.0/400;
    std::string robot_name = "solo12";
    MPC mpc(robot_name, dT, 1);

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

    gen_pose.joint_pos.resize(12);
    gen_pose.joint_vel.resize(12);

    gen_pose.joint_pos << 0.0, 0.3, -0.6, 0.0, 0.3, -0.6, 0.0, -0.3, 0.6, 0.0, -0.3, 0.6;
    gen_pose.joint_vel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    gen_pose.contact_feet_names = {"FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"};

    //Eigen::VectorXd q = mpc.get_robot().clik_alg(q_init, gen_pose, mpc.get_robot().get_robot_model(), mpc.get_robot().get_robot_data());
    //std::cout <<"q inverse:\n" <<q <<"\n" <<std::endl;

    GeneralizedPoseWithTime gen_pose_time;
    gen_pose_time.gen_pose = gen_pose;
    gen_pose_time.time = 0;

    GeneralizedPosesWithTime gen_poses_time;
    std::vector<GeneralizedPoseWithTime> gen_poses = {gen_pose_time};
    gen_poses_time.generalized_poses_with_time = gen_poses;

    std::cout <<"Prova di stampa\n";

    std::vector<Eigen::VectorXd> sol(1);
    sol = mpc.solve_MPC(q_init, qdot_init, gen_poses_time);

    std::cout <<"Stampa di fine ottimizzazione coppie\n";

    std::vector<Eigen::VectorXd> pid_gains(3);
    pid_gains = mpc.tune_gains(q_init, qdot_init, gen_poses_time);
    for (int i=0; i<gen_poses_time.generalized_poses_with_time.size(); i++){
        std::cout <<"Pid gains: \n" <<pid_gains[i] <<"\n";
    }


}