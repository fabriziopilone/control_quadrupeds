#pragma once

#include "hierarchical_optimization/HO.hpp"
#include "model_predictive_control/Robot.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

struct GeneralizedPose {
    // Base linear quantities
    Eigen::Vector3d base_acc = {0, 0, 0};
    Eigen::Vector3d base_vel = {0, 0, 0};
    Eigen::Vector3d base_pos = {0, 0, 0};

    // Base angular quantities
    Eigen::Vector3d base_angvel = {0, 0, 0};
    Eigen::Vector4d base_quat = {0, 0, 0, 1};

    // Swing feet linear quantities
    Eigen::VectorXd feet_acc = {};
    Eigen::VectorXd feet_vel = {};
    Eigen::VectorXd feet_pos = {};

    // List of feet names in contact with the ground
    std::vector<std::string> contact_feet_names;
};

struct GeneralizedPoseWithTime{
    GeneralizedPose gen_pose;
    float time;
};

struct GeneralizedPosesWithTime{
    std::vector<GeneralizedPoseWithTime> generalized_poses_with_time;
};

class MPC{

    public:

    // Constructors
        MPC(std::string robot_name, double dT);

    // Getters
        int get_steps(){return this->mpc_step_horizon;};
        double get_dT(){return this->dT;};
        Robot get_robot(){return this->robot;};

    // Setters
        void set_steps(int steps){this->mpc_step_horizon = steps;};

        std::vector<Eigen::VectorXd> solve_MPC(Eigen::VectorXd q, Eigen::VectorXd q_dot, GeneralizedPosesWithTime gen_poses);
        std::vector<Eigen::VectorXd> tune_gains(Eigen::VectorXd, Eigen::VectorXd, GeneralizedPosesWithTime);

    private:
    // Private methods
        std::vector<Task> create_tasks(std::vector<std::string>, GeneralizedPosesWithTime, Eigen::VectorXd, Eigen::VectorXd);
        Task dynamic_constraint(pinocchio::Model, pinocchio::Data, Eigen::VectorXd, Eigen::VectorXd);
        Task torque_limits_constraint();
        Task contact_constraint();
        Task motion_tracking_constraint(GeneralizedPosesWithTime);
        Task friction_constraint(GeneralizedPosesWithTime);
        int resolveOption(std::string);

    // Private attributes
        Robot robot;
        HO hierarchical_optimization;
        std::string robot_name;     // Name of the robot
        int mpc_step_horizon;       // Number of step of the Model Predictive Control algorithm
        double dT;      // Sample time
        double mu;      // Friction coefficient

        std::vector<std::string> task_request;      // Vector of task for the optimization
};