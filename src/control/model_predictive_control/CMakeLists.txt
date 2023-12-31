cmake_minimum_required(VERSION 3.8)
project(model_predictive_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_package(hierarchical_optimization REQUIRED)

set(LIBRARY_DEPENDENCIES
	Eigen3
	pinocchio
	
	hierarchical_optimization
)

add_library(${PROJECT_NAME} SHARED src/MPC.cpp src/Robot.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${LIBRARY_DEPENDENCIES})
#target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${LIBRARY_DEPENDENCIES})

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

add_executable(test_MPC test/test_mpc.cpp)
target_include_directories(test_MPC PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_MPC PUBLIC ${PROJECT_NAME})
ament_target_dependencies(test_MPC PUBLIC Eigen3 hierarchical_optimization)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
