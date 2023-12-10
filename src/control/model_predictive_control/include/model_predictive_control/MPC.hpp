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
        MPC(std::string robot_name, double dT, int mpc_step_horizon);

    // Getters
        int get_steps(){return this->mpc_step_horizon;};
        double get_dT(){return this->dT;};
        Robot get_robot(){return this->robot;};
        std::vector<std::string> get_generic_feet_names(){return robot.get_feet_names();};
        Eigen::Vector3d get_Kp_pos(){return this->Kp_pos;};
        Eigen::Vector3d get_Kd_pos(){return this->Kd_pos;};
        Eigen::Vector3d get_Kp_ang(){return this->Kp_ang;};
        Eigen::Vector3d get_Kd_ang(){return this->Kd_ang;};
        Eigen::Vector3d get_Kp_s_pos(){return this->Kp_s_pos;};
        Eigen::Vector3d get_Kd_s_pos(){return this->Kd_s_pos;};
        Eigen::VectorXd get_feet_positions();
        Eigen::VectorXd get_feet_velocities(Eigen::VectorXd v);

    // Setters
        void set_steps(int steps){this->mpc_step_horizon = steps;};
        void set_tau_max(double tau_max){this->robot.set_tau_max(tau_max);};
        void set_tau_min(double tau_min){this->robot.set_tau_min(tau_min);};
        void set_f_max(double f_max){this->robot.set_f_max(f_max);};
        void set_f_min(double f_min){this->robot.set_f_min(f_min);};
        void set_Kp_pos(Eigen::Vector3d Kp_pos){this->Kp_pos = Kp_pos;};
        void set_Kd_pos(Eigen::Vector3d Kd_pos){this->Kd_pos = Kd_pos;};
        void set_Kp_ang(Eigen::Vector3d Kp_ang){this->Kp_ang = Kp_ang;};
        void set_Kd_ang(Eigen::Vector3d Kd_ang){this->Kd_ang = Kd_ang;};
        void set_Kp_s_pos(Eigen::Vector3d Kp_s_pos){this->Kp_s_pos = Kp_s_pos;};
        void set_Kd_s_pos(Eigen::Vector3d Kd_s_pos){this->Kd_s_pos = Kd_s_pos;};

        void set_mu(double mu){this->mu = mu;};
        void set_tau(Eigen::VectorXd tau){this->robot.set_optimal_actual_input(tau);};

        std::vector<Eigen::VectorXd> solve_MPC(Eigen::VectorXd q, Eigen::VectorXd q_dot, GeneralizedPosesWithTime gen_poses);
        std::vector<Eigen::VectorXd> tune_gains(Eigen::VectorXd, Eigen::VectorXd, GeneralizedPosesWithTime);

    private:
    // Private methods
        void compute_all(Eigen::VectorXd, Eigen::VectorXd, GeneralizedPosesWithTime);
        std::vector<Task> create_tasks(std::vector<std::string>, GeneralizedPosesWithTime, Eigen::VectorXd, Eigen::VectorXd);
        Task contact_constraint(GeneralizedPosesWithTime des_gen_poses);
        Task dynamic_constraint(GeneralizedPosesWithTime, Eigen::VectorXd, Eigen::VectorXd);
        Task torque_limits_constraint();
        Task motion_tracking_constraint(GeneralizedPosesWithTime);
        Task motion_tracking_base_constraint(GeneralizedPosesWithTime, Eigen::VectorXd, Eigen::VectorXd);
        Task motion_tracking_swing_feet_constraint(GeneralizedPosesWithTime, Eigen::VectorXd, Eigen::VectorXd);
        Task friction_constraint(GeneralizedPosesWithTime);
        Task effort_minimization();
        int resolveOption(std::string);

    // Private attributes
        Robot robot;
        HO hierarchical_optimization;
        std::string robot_name;     // Name of the robot
        int mpc_step_horizon;       // Number of step of the Model Predictive Control algorithm
        std::vector<Eigen::VectorXd> q_propagated;
        std::vector<Eigen::VectorXd> v_propagated;
        std::vector<Eigen::MatrixXd> Jc_in_time;
        std::vector<Eigen::MatrixXd> Jc_dot_in_time;
        Eigen::MatrixXd Minv;
        double dT;      // Sample time
        double mu;      // Friction coefficient
        Eigen::Vector3d Kp_pos = {100., 100., 100.};       // 100
        Eigen::Vector3d Kd_pos = {10., 10., 10.};        // 10
        Eigen::Vector3d Kp_ang = {150., 150, 150.};       //150
        Eigen::Vector3d Kd_ang = {35., 35., 35.};        // 35
        Eigen::Vector3d Kp_s_pos = {900., 900., 900.};     // 900
        Eigen::Vector3d Kd_s_pos = {30., 30., 30.,};      // 30
        bool flag = false;

        int number_of_wolves = 20;      // Number of particles for the tuning of the PID

        std::vector<std::string> task_request;      // Vector of task for the optimization
};
