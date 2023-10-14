# SOLO 12 Robot Description

## Overview

This package contains the robot description of the SOLO 12 robot.

![SOLO 12 Robot Description](images/solo12.png)

## Usage

To visualize and debug the robot description, start the standalone visualization (note that you have to provide the following additional dependencies: `joint_state_publisher`, `joint_state_publisher_gui`, `robot_state_publisher`, `rviz2`, `xacro`):

    ros2 launch solo_description standalone.launch.py

The `robot_model` exec_depend is necessary to find the config file containing the parameters of the controllers implemented in https://github.com/ddebenedittis/control_quadrupeds_soft_contacts. The `$(find robot_control)/...` in the `gazebo.xacro` file or the whole `gazebo.xacro` file can be removed if unnecessary.