#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <Eigen/Core>

class Robot{

    public:
    Robot();
    Robot(std::string robot_name);

    //Getters
    std::string get_name(){return this->robot_name;};
    pinocchio::Model get_robot_model(){return this->robot_model;};
    pinocchio::Data get_robot_data(){return this->robot_data;};
    int get_state_dim(){return this->state_dim;};
    int get_contact_feet_dim(){return this->contact_feet_dim;};
    Eigen::VectorXd get_q(){return this->q;};
    Eigen::VectorXd get_qdot(){return this->q_dot;};
    std::vector<Eigen::VectorXd> get_optimal_input(){return this->optimal_input;};
    double get_tau_min(){return this->tau_min;};
    double get_tau_max(){return this->tau_max;};
    double get_f_max(){return this->f_max;};
    double get_f_min(){return this->f_min;};

    //Setters
    void set_q(Eigen::VectorXd q){this->q = q;};
    void set_qdot(Eigen::VectorXd q_dot){this->q_dot = q_dot;}
    void set_ground_feet_names(std::vector<std::string> ground_feet_names){
        this->ground_feet_names = ground_feet_names;
        this->contact_feet_dim = ground_feet_names.size();
        };
    void set_optimal_input(std::vector<Eigen::VectorXd> optimal_input){this->optimal_input = optimal_input;};

    //Methods
    std::vector<Eigen::VectorXd> compute_dynamics(Eigen::VectorXd q, Eigen::VectorXd q_dot, Eigen::VectorXd tau, double dT);
    void get_Jc(Eigen::MatrixXd& Jc);

    private:
    std::string robot_name;
    pinocchio::Model robot_model;
    pinocchio::Data robot_data;

    int state_dim;
    int contact_feet_dim;
    double tau_max = 80;   // ?????????????
    double tau_min = -80;  // ?????????????
    double f_max = 350;     // ???????????
    double f_min = 40;      // ????????????
    double dT;

    std::vector<std::string> ground_feet_names;

    Eigen::VectorXd q;
    Eigen::VectorXd q_dot;
    std::vector<Eigen::VectorXd> optimal_input;
};