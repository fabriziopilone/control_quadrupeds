# ==============================================================================
#                             PROJECT CONFIGURATION                             
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(quadprog)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



# ==============================================================================
#                               FIND DEPENDENCIES                               
# ==============================================================================

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)



# ==============================================================================
#                                 ADD LIBRARIES                                 
# ==============================================================================

set(LIBRARY_NAME ${PROJECT_NAME})

add_library(${LIBRARY_NAME} SHARED
    src/quadprog.cpp
    src/solve.QP.cpp
    src/qr-update.cpp
    src/linear-algebra.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${LIBRARY_NAME} Eigen3)

ament_export_targets(${LIBRARY_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)
# install(
#     TARGETS ${PROJECT_NAME}
#     EXPORT ${PROJECT_NAME}_targets
#     LIBRARY DESTINATION lib/${PROJECT_NAME}
#     ARCHIVE DESTINATION lib/${PROJECT_NAME}
#     RUNTIME DESTINATION lib/${PROJECT_NAME}
#     INCLUDES DESTINATION include/${PROJECT_NAME}
# )
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)



# ==============================================================================
#                                  ADD TESTS                                 
# ==============================================================================

add_executable(Test test/test.cpp)

target_include_directories(Test PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(Test PUBLIC ${LIBRARY_NAME})
ament_target_dependencies(Test PUBLIC Eigen3)



if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
