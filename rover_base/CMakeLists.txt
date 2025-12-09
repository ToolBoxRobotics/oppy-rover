cmake_minimum_required(VERSION 3.0.2)
project(rover_base)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  rover_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/base_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
