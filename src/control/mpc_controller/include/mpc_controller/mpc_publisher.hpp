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
            void publish_all(
                Eigen::VectorXd joints_acceleration, Eigen::VectorXd torques, Eigen::Vector forces,
                Eigen::VectorXd feet_position, Eigen::VectorXd feet_velocites
            );

        private:
        int mpc_step_horizon = 1;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joints_acceleration_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torques_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feet_position_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr feet_velocities_publisher;

    };
}