#pragma once

#include "controller_interface/controller_interface.hpp"
#include "model_predictive_control/MPC.hpp"
#include "mpc_controller/mpc_publisher.hpp"

#include "gazebo_msgs/msg/link_states.hpp"
#include "generalized_pose_msgs/msg/generalized_pose.hpp"
#include "generalized_pose_msgs/msg/generalized_poses_with_time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <stdio.h>
namespace mpc_controller{

// MPC CONTROLLER

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MPCController : public controller_interface::ControllerInterface{

    public:
        MPCController();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& /*period*/
        ) override;

        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    
    private:
        int mpc_step_horizon = 2;
        double Kp_PD;
        double Kd_PD;
        MPC mpc;
        std::vector<std::string> joint_names;

        Eigen::VectorXd q = Eigen::VectorXd::Zero(12);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(12);
        std::vector<Eigen::VectorXd> tau;
        std::vector<Eigen::VectorXd> pid_gains;
        GeneralizedPose des_gen_pose;
        GeneralizedPosesWithTime des_gen_poses_single;
        GeneralizedPosesWithTime des_gen_poses;
        Eigen::VectorXd q_init = Eigen::VectorXd::Zero(12);

        // Subscriptions
        rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr joint_state_subscription = nullptr;
        rclcpp::Subscription<generalized_pose_msgs::msg::GeneralizedPosesWithTime>::SharedPtr des_gen_poses_subscription = nullptr;
        rclcpp::Subscription<generalized_pose_msgs::msg::GeneralizedPose>::SharedPtr des_gen_pose_subscription = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr estimated_pose_subscription_ = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr estimated_twist_subscription_ = nullptr;

        bool logging = true;

        // Publishers
        std::shared_ptr<MPCPublisher> logger = nullptr;

        double init_time = 1;

};
}
