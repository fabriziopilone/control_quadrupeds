#include "model_predictive_control/Robot.hpp"

Robot::Robot(){}

Robot::Robot(std::string robot_name){
    this->robot_name = robot_name;

    const std::string urdf_filename = std::string("/robot/solo_description/urdf/solo12.urdf");
    this->robot_model = pinocchio::urdf::buildModel(urdf_filename, robot_model);    // Load the urdf model
    this->robot_data = pinocchio::Data(robot_model);    // Create data required by the algorithms
    this->state_dim = this->robot_model.nv;
    this->contact_feet_dim = 0;
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

void Robot::get_Jc(Eigen::MatrixXd& Jc)
{
    // Initialize a temp Jacobian that must be used to store the contact jacobian of a contact foot.
    // Jc is the stack of J_temp of all the contact feet.
    Eigen::MatrixXd J_temp(6, robot_model.nv);

    // Compute the stack of the contact Jacobians
    for (int i = 0; i < contact_feet_dim; i++) {
        pinocchio::FrameIndex frame_id = robot_model.getFrameId(ground_feet_names[i]);

        J_temp.setZero();
        
        pinocchio::getFrameJacobian(robot_model, robot_data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp);

        Jc.block(3*i, 0, 3, robot_model.nv) = J_temp.topRows(3);
    }
}