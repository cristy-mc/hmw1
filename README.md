🤖 HMW1 – Armando Robot

🧩 Overview
This project implements Homework 1 of the Robotics Lab 2025 course.
The goal is to build, simulate, and control a 4-DOF robotic manipulator (Armando) within the ROS 2 + Gazebo environment.

🔨 Build
Clone this package in the src folder of the ROS 2 workspace. Check for missing dependencies:
$ sudo apt-install joint-state-publisher
Build the new packages:
$ colcon build --packages-select armando_description armando_gazebo armando_controller

🚀 Launch Instructions
1️⃣ Visualize the robot in RViz
$ros2 launch armando_description armando_display.launch.py

2️⃣ Spawn the robot in Gazebo
ros2 launch armando_gazebo armando_world.launch.py

🎮 How to Control the Robot
Launch the nodes for position controller:
$ros2 launch armando_gazebo armando_world.launch.py controller_type:=position
$ros2 launch armando_controller armando_controller.launch.py controller_type:=position

Launch the nodes for trajectory controller:
$ros2 launch armando_gazebo armando_world.launch.py controller_type:=trajectory
$ros2 launch armando_controller armando_controller.launch.py controller_type:=trajectory

If no argument is specified, the position controller is loaded by default.
