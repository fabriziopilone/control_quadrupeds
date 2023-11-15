#pragma once

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include <Eigen/Core>

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

    // Joint position and velocity
    Eigen::VectorXd joint_pos = {};
    Eigen::VectorXd joint_vel = {};

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
    std::vector<Eigen::VectorXd> get_optimal_torques(){return this->optimal_torques;};
    std::vector<Eigen::VectorXd> get_optimal_contact_forces(){return this->optimal_contact_forces;};
    std::vector<Eigen::VectorXd> get_optimal_input(){return this->optimal_input;};
    double get_tau_min(){return this->tau_min;};
    double get_tau_max(){return this->tau_max;};
    double get_f_max(){return this->f_max;};
    double get_f_min(){return this->f_min;};
    std::vector<std::string> get_feet_names(){return this->feet_names;};
    std::vector<std::string> get_short_feet_names(){return this->short_feet_names;};

    //Setters
    void set_q(Eigen::VectorXd q){this->q = q;};
    void set_qdot(Eigen::VectorXd q_dot){this->q_dot = q_dot;}
    void set_ground_feet_names(std::vector<std::string> ground_feet_names){
        this->ground_feet_names = ground_feet_names;
        //this->contact_feet_dim = ground_feet_names.size();
        };
    void set_optimal_input(std::vector<Eigen::VectorXd> optimal_input){this->optimal_input = optimal_input;};
    void set_optimal_torques(std::vector<Eigen::VectorXd> optimal_torques){this->optimal_torques = optimal_torques;};
    void set_optimal_contact_forces(std::vector<Eigen::VectorXd> optimal_contact_forces){this->optimal_contact_forces = optimal_contact_forces;};
    void set_optimal_actual_input(Eigen::VectorXd optimal_input){this->optimal_input[0] = optimal_input;};
    void set_tau_max(double tau_max){
        std::cout <<"Robot setting tau_max to " <<tau_max <<std::endl;
        this->tau_max = tau_max;
        };
    void set_tau_min(double tau_min){this->tau_min = tau_min;};
    void set_f_max(double f_max){this->f_max = f_max;};
    void set_f_min(double f_min){this->f_min = f_min;};

    //Methods
    std::vector<Eigen::VectorXd> compute_dynamics(Eigen::VectorXd q, Eigen::VectorXd q_dot, Eigen::VectorXd tau, double dT);
    void compute_second_order_FK(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
    void get_Jc(Eigen::MatrixXd& Jc, Eigen::VectorXd q, std::vector<std::string> ground_feet_names, std::vector<int> contact_feet_index);
    void get_Jb(Eigen::MatrixXd& Jb, Eigen::VectorXd q);
    void get_Js(Eigen::MatrixXd& Js, std::vector<std::string>, std::vector<int>);
    void compute_EOM(const Eigen::VectorXd& q, const Eigen::VectorXd& v);
    void compute_terms(Eigen::VectorXd);
    Eigen::VectorXd clik_alg(Eigen::VectorXd, GeneralizedPose, pinocchio::Model, pinocchio::Data);

    private:
    std::string robot_name;
    pinocchio::Model robot_model;
    pinocchio::Data robot_data;

    int state_dim;
    int contact_feet_dim = 4;  // 3 components of force for each leg
    double tau_max = 10;   // ?????????????
    double tau_min = -2.7;  // ?????????????
    double f_max = 15.;     // ???????????
    double f_min = 2.;      // ????????????
    double dT;

    std::vector<std::string> ground_feet_names = {"LF", "RF", "LH", "RH"};
    std::vector<std::string> short_feet_names = {"LF", "RF", "LH", "RH"};
    std::vector<std::string> feet_names = {"FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"};

    Eigen::VectorXd q;
    Eigen::VectorXd q_dot;
    std::vector<Eigen::VectorXd> optimal_torques;
    std::vector<Eigen::VectorXd> optimal_contact_forces;
    std::vector<Eigen::VectorXd> optimal_input;
};
