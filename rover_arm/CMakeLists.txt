cmake_minimum_required(VERSION 3.0.2)
project(rover_arm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  trajectory_msgs
  std_msgs
  rover_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/arm_bridge.py
  src/arm_homing_supervisor.py
  src/arm_trajectory_executor.py
  src/arm_interactive_marker.py
  src/gripper_interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
