#include "model_predictive_control/Robot.hpp"
using namespace Eigen;

Robot::Robot(){}

Robot::Robot(std::string robot_name){
    this->robot_name = robot_name;

    const std::string urdf_filename = std::string("robot/solo_description/urdf/solo12.urdf");
    this->robot_model = pinocchio::urdf::buildModel(urdf_filename, robot_model);    // Load the urdf model
    this->robot_data = pinocchio::Data(robot_model);    // Create data required by the algorithms
    this->state_dim = this->robot_model.nv;
    this->contact_feet_dim = 4;
    this->q = Eigen::VectorXd::Zero(this->state_dim);
    this->q_dot = Eigen::VectorXd::Zero(this->state_dim);

    this->dT = 1.0/400;

}

std::vector<Eigen::VectorXd> Robot::compute_dynamics(Eigen::VectorXd q, Eigen::VectorXd q_dot, Eigen::VectorXd tau, double dT){

    Eigen::VectorXd q_ddot = pinocchio::aba(this->robot_model, this->robot_data, q, q_dot, tau);
    Eigen::VectorXd q_dot_updated = q_dot + dT*q_ddot;
    Eigen::VectorXd q_updated = pinocchio::integrate(this->robot_model, q, q_dot_updated);
    std::vector<Eigen::VectorXd> joint_state_propagated(2);
    joint_state_propagated[0] = q_updated;
    joint_state_propagated[1] = q_dot_updated;
    return joint_state_propagated;
}

void Robot::get_Jc(Eigen::MatrixXd& Jc, Eigen::VectorXd q)
{
    // Initialize a temp Jacobian that must be used to store the contact jacobian of a contact foot.
    // Jc is the stack of J_temp of all the contact feet.

    pinocchio::computeJointJacobians(robot_model, robot_data, q);
    pinocchio::framesForwardKinematics(robot_model, robot_data, q);

    Eigen::MatrixXd J_temp = MatrixXd::Zero(6, robot_model.nv);

    // Compute the stack of the contact Jacobians
    for (int i = 0; i < contact_feet_dim; i++) {
        std::cout <<"Stampa ground_feet_names\n" <<ground_feet_names[i] <<"\n";
        pinocchio::FrameIndex frame_id = robot_model.getFrameId(feet_names[i]);
        std::cout <<"frame_id:\n" <<frame_id <<"\n";

        J_temp.setZero();

        //std::cout <<"Existance frame:\n" << pinocchio::Model::existFrame(feet_names[i]);
        
        pinocchio::getFrameJacobian(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        //std::cout <<"J_temp:\n" <<J_temp <<"\n";

        Jc.block(3*i, 0, 3, robot_model.nv) = J_temp.topRows(3);
    }
}

void Robot::compute_terms(Eigen::VectorXd q){
    // Update the joint placements
    //pinocchio::forwardKinematics(robot_model, robot_data, q);

    // Computes the full model Jacobian
    pinocchio::computeJointJacobians(robot_model, robot_data, q);

    // Update the frame placements
    //pinocchio::updateFramePlacements(robot_model, robot_data);

    pinocchio::framesForwardKinematics(robot_model, robot_data, q);

    // Compute the upper part of the joint space inertia matrix
    //pinocchio::crba(robot_model, robot_data, q);
    //data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Compute the nonlinear effects vector (Coriolis, centrifugal and gravitational effects)
    //pinocchio::nonLinearEffects(robot_model, robot_data, q, v);
}