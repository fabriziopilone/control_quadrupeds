cmake_minimum_required(VERSION 3.8)
project(hierarchical_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(quadprog REQUIRED)

# Add Libraries

add_library(${PROJECT_NAME} SHARED src/Task.cpp src/HO.cpp)

#target_link_libraries(hierarchical_optimization)
#include_directories(include)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} Eigen3 quadprog)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
#ament_export_dependencies(Eigen3)
ament_export_dependencies(Eigen3 quadprog)

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


#add_executable(test1 test/test1.cpp)
#target_include_directories(test1 PUBLIC
#  ${EIGEN3_INCLUDE_DIR}
#)
#target_link_libraries(test1 PUBLIC ${PROJECT_NAME})
# ament_target_dependencies(test1 ${PROJECT_NAME})

####################
#     TESTING	   #
####################

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_test test/gtest.cpp)
  
      target_include_directories(${PROJECT_NAME}_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        test/
    )
    ament_target_dependencies(${PROJECT_NAME}_test Eigen3 quadprog)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  
endif()

ament_package()
