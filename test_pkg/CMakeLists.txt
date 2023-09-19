cmake_minimum_required(VERSION 3.8)
project(test_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hierarchical_optimization REQUIRED)
find_package(Eigen3 REQUIRED)

#include_directories(home/fabrizio/Github/control_quadrupeds/src/hierarchical_optimization)
set(path "${CMAKE_BINARY_DIR}")
message(BINARY_DIR "${CMAKE_SOURCE_DIR}")
cmake_path(GET path PARENT_PATH PARENT_DIR)
cmake_path(GET PARENT_DIR PARENT_PATH PARENT_DIR)
cmake_path(GET PARENT_DIR PARENT_PATH PARENT_DIR)
message(PARENT_DIR : "${PARENT_DIR}")

#add_library(hierarchical_optimization SHARED IMPORTED GLOBAL)
#set_target_properties( hierarchical_optimization PROPERTIES IMPORTED_LOCATION ${PARENT_DIR}/src/hierarchical_optimization/install/hierarchical_optimization/libhierarchical_optimization.so )

#link_directories(${CMAKE_BINARY_DIR}/hierarchical_optimization})
add_executable(test1 src/test1.cpp)
#target_include_directories(test1 PUBLIC "${PARENT_DIR}/src/hierarchical_optimization/include")
#target_link_directories(test1 PUBLIC "${PARENT_DIR}/src/hierarchical_optimization")
#target_link_libraries(test1 hierarchical_optimization)

#target_include_directories(test1 PUBLIC GLOBAL "/hierarchical_optimization/include")
#target_link_directories(test1 PUBLIC GLOBAL "/hierarchical_optimization")
target_link_libraries(test1 hierarchical_optimization)

ament_target_dependencies(test1 Eigen3)

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
