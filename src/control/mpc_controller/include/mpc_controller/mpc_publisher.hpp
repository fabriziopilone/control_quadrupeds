#pragma once

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rviz_legged_msgs/msg/friction_cones.hpp"
#include "rviz_legged_msgs/msg/wrenches_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

namespace mpc_controller{

    class MPCPublisher: public rclcpp::Node{

        public:
            MPCPublisher();
            static inline void publish_float64_multi_array(
                const Eigen::VectorXd& vector,
                const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher);
            void publish_all(
                Eigen::VectorXd torques, std::vector<Eigen::VectorXd> pid_gains, Eigen::VectorXd q_opt, Eigen::VectorXd v_opt, Eigen::VectorXd des_base_state,
                Eigen::VectorXd feet_positions, Eigen::VectorXd feet_velocities
            );

        private:
        int mpc_step_horizon = 1;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr q_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr v_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr des_base_state_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_acceleration_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torques_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feet_position_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feet_velocities_publisher;
        //rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pid_gains_publisher;

    };
}