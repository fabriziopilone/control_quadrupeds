#pragma once

#include "hierarchical_optimization/HO.hpp"
#include "model_predictive_control/Robot.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

class MPC{

    public:

    // Constructors
        MPC(std::string robot_name, double dT);

    // Getters
        int get_steps(){return this->mpc_step_horizon;};
        double get_dT(){return this->dT;};
        Robot get_robot(){return this->robot;};
        std::vector<std::string> get_generic_feet_names(){return robot.get_feet_names();};

    // Setters
        void set_steps(int steps){this->mpc_step_horizon = steps;};
        void set_mu(double mu){this->mu = mu;};

        std::vector<Eigen::VectorXd> solve_MPC(Eigen::VectorXd q, Eigen::VectorXd q_dot, GeneralizedPosesWithTime gen_poses);
        std::vector<Eigen::VectorXd> tune_gains(Eigen::VectorXd, Eigen::VectorXd, GeneralizedPosesWithTime);

    private:
    // Private methods
        std::vector<Task> create_tasks(std::vector<std::string>, GeneralizedPosesWithTime, Eigen::VectorXd, Eigen::VectorXd);
        Task dynamic_constraint(GeneralizedPosesWithTime, pinocchio::Model, pinocchio::Data, Eigen::VectorXd, Eigen::VectorXd);
        Task torque_limits_constraint();
        Task contact_constraint();
        Task motion_tracking_constraint(GeneralizedPosesWithTime);
        Task motion_tracking_base_constraint(GeneralizedPosesWithTime, Eigen::VectorXd, Eigen::VectorXd);
        Task motion_tracking_swing_feet_constraint(GeneralizedPosesWithTime);
        Task friction_constraint(GeneralizedPosesWithTime);
        int resolveOption(std::string);

    // Private attributes
        Robot robot;
        HO hierarchical_optimization;
        std::string robot_name;     // Name of the robot
        int mpc_step_horizon;       // Number of step of the Model Predictive Control algorithm
        double dT;      // Sample time
        double mu;      // Friction coefficient

        int number_of_wolves = 20;      // Number of particles for the tuning of the PID

        std::vector<std::string> task_request;      // Vector of task for the optimization
};
