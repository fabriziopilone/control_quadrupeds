#include "mpc_controller/mpc_publisher.hpp"

namespace mpc_controller{

// Constructor
    MPCPublisher::MPCPublisher() : Node("MPC_publisher"){
        this->q_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/q_opt", 1);

        this->v_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "logging/v_opt", 1);

        this->des_base_state_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/des_base_state", 1);

        this->joints_acceleration_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/optimal_joints_acceleration", this->mpc_step_horizon);

        this->torques_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/optimal_torques", this->mpc_step_horizon);

        this->forces_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/optimal_forces", this->mpc_step_horizon);

        this->feet_position_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/feet_position", 1);

        this->feet_velocities_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/logging/feet_velocities", 1);
            

        //this->pid_gains_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        //    "/logging/pid_gains", 1);

    }

   inline void MPCPublisher::publish_float64_multi_array(
    const Eigen::VectorXd& vector,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher)
{
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = std::vector<double>(vector.data(), vector.data() + vector.size());
    publisher->publish(message);
}

   void MPCPublisher::publish_all(Eigen::VectorXd torques, std::vector<Eigen::VectorXd> pid_gains, Eigen::VectorXd q_opt, Eigen::VectorXd v_opt, Eigen::VectorXd des_base_state,
   Eigen::VectorXd feet_positions, Eigen::VectorXd feet_velocities){
    auto message = std_msgs::msg::Float64MultiArray();

    // Publish torques
    publish_float64_multi_array(torques, torques_publisher);

    // Publish q
    publish_float64_multi_array(q_opt, q_publisher);

    // Publish v
    publish_float64_multi_array(v_opt, v_publisher);

    // Publish des_base_state
    publish_float64_multi_array(des_base_state, des_base_state_publisher);
    
    // Publish feet_positions
    publish_float64_multi_array(feet_positions, feet_position_publisher);

    // Publish feet velocities
    publish_float64_multi_array(feet_velocities, feet_velocities_publisher);
        

    // Publish PID gains
        //message.data = std::vector<Eigen::Vector>(pid_gains.data(), pid_gains.data()+pid_gains.size());
        //message.data = std::vector<double>(pid_gains.data(), pid_gains.data()+pid_gains.size());
        //this->pid_gains_publisher->publish(message);
   }
}
