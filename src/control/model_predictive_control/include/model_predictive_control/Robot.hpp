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
    std::vector<std::string> get_feet_names(){return this->feet_names;};

    //Setters
    void set_q(Eigen::VectorXd q){this->q = q;};
    void set_qdot(Eigen::VectorXd q_dot){this->q_dot = q_dot;}
    void set_ground_feet_names(std::vector<std::string> ground_feet_names){
        this->ground_feet_names = ground_feet_names;
        //this->contact_feet_dim = ground_feet_names.size();
        };
    void set_optimal_input(std::vector<Eigen::VectorXd> optimal_input){this->optimal_input = optimal_input;};
    void set_tau_max(double tau_max){this->tau_max = tau_max;};
    void set_tau_min(double tau_min){this->tau_min = tau_min;};
    void set_f_max(double f_max){this->f_max = f_max;};
    void set_f_min(double f_min){this->f_min = f_min;};

    //Methods
    std::vector<Eigen::VectorXd> compute_dynamics(Eigen::VectorXd q, Eigen::VectorXd q_dot, Eigen::VectorXd tau, double dT);
    void get_Jc(Eigen::MatrixXd& Jc, Eigen::VectorXd q);
    void compute_terms(Eigen::VectorXd);

    private:
    std::string robot_name;
    pinocchio::Model robot_model;
    pinocchio::Data robot_data;

    int state_dim;
    int contact_feet_dim = 4;  // 3 components of force for each leg
    double tau_max = 80;   // ?????????????
    double tau_min = -80;  // ?????????????
    double f_max = 350;     // ???????????
    double f_min = 40;      // ????????????
    double dT;

    std::vector<std::string> ground_feet_names = {"LF", "RF", "LH", "RH"};
    std::vector<std::string> feet_names = {"FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"};

    Eigen::VectorXd q;
    Eigen::VectorXd q_dot;
    std::vector<Eigen::VectorXd> optimal_input;
};