#include <gtest/gtest.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadMPCController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::minimal_robot_urdf),
    executor, "test_controller_manager");

  ASSERT_NE(
    cm.load_controller("test_mpc_controller", "mpc_controller/MPCController"),
    nullptr);

  rclcpp::shutdown();
}
