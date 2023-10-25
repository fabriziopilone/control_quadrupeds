cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(gazebo_msgs REQUIRED)
find_package(generalized_pose_msgs REQUIRED)
find_package(rviz_legged_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(model_predictive_control REQUIRED)


set(PROJECT_DEPENDENCIES
	controller_interface
	hardware_interface
	rclcpp
	rclcpp_lifecycle
	gazebo_msgs
	generalized_pose_msgs
	rviz_legged_msgs
	sensor_msgs
	model_predictive_control
)
add_library(${PROJECT_NAME} SHARED src/mpc_controller.cpp src/mpc_publisher.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${PROJECT_DEPENDENCIES})

pluginlib_export_plugin_description_file(controller_interface mpc_controller.xml)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

install(
	DIRECTORY include/
	DESTINATION include
)

install(
	TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}_targets
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_test test/test_load_mpccontroller.cpp)
  
  target_include_directories(${PROJECT_NAME}_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        test/
    )
  
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
