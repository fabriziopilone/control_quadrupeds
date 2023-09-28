#pragma once

#include "hierarchical_optimization/HO.hpp"
#include "model_predictive_control/Robot.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"

#include "generalized_pose_msgs/msg/generalized_pose.hpp"
#include "generalized_pose_msgs/msg/generalized_poses_with_time.hpp"

class MPC{

    public:

    // Constructors
        MPC(std::string, double);

    // Getters
        int get_steps(){return this->mpc_step_horizon;};
        double get_dT(){return this->dT;};

    // Setters
        void set_steps(int steps){this->mpc_step_horizon = steps;};


        std::vector<Eigen::VectorXd> solve_MPC(Eigen::VectorXd q, Eigen::VectorXd q_dot, GeneralizedPose gen_pose);

    private:
    // Private methods
        std::vector<Task> create_tasks(std::vector<std::string>);
        Task dynamic_constraint(pinocchio::Model, pinocchio::Data);
        Task torque_limits_constraint();
        Task contact_constraint();
        Task motion_tracking_constraint(GeneralizedPosesWithTime);
        Task friction_constraint();
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