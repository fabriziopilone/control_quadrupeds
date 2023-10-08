#include "mpc_controller/mpc_publisher.hpp"

namespace mpc_controller{

// Constructor
    MPCPublisher::MPCPublisher() : Node("MPC_publisher"){
        this->joints_acceleration_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/optimal_joints_acceleration", this->mpc_step_horizon);

        this->torques_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/optimal_torques", this->mpc_step_horizon);

        this->forces_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/optimal_forces", this->mpc_step_horizon);

        this->feet_position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/feet_position", this->mpc_step_horizon);

        this->feet_velocities_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/feet_velocities", this->mpc_step_horizon);

        this->pid_gains_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/pid_gains", 1);

    }

    void MPCPublisher::publish_all(
        Eigen::VectorXd joints_acceleration, Eigen::VectorXd torques, Eigen::Vector forces,
        Eigen::VectorXd feet_position, Eigen::VectorXd feet_velocites, std::vector<Eigen::VectorXd> pid_gains){

        auto message = std_msgs::msg::Float64MultiArray();

        // Publish joint accelerations
        message.data = std::vector<double>(joints_acceleration.data(), joints_acceleration.data() + joints_acceleration.size());
        this->joints_acceleration_publisher->publish(message);

        // Publish torques
        message.data = std::vector<double>(torques.data(), torques.data()+)torques.size();
        this->torques_publisher->publish(message);

        // Publish forces
        message.data = std::vector<double>(forces.data(), forces.data()+forces.size());
        this->forces_publisher->publish(message);

        // Publish feet position
        message.data = std::vector<double>(feet_position.data(), feet_position.data()+feet_position.size());
        this->feet_position_publisher->publish(message);

        // Publish feet velocities
        message.data = std::vector<double>(feet_velocites.data(), feet_velocites.data()+feet_velocites.size());
        this->feet_velocities_publisher->publish(message);

        // Publish PID gains
        message.data = std::vector<Eigen::Vector>(pid_gains.data(), pid_gains.data()+pid_gains.size());
        this->pid_gains_publisher->publish(message);
    }
}