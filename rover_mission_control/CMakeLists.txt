cmake_minimum_required(VERSION 3.0.2)
project(rover_mission_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  rover_msgs
  python_qt_binding
  cv_bridge
  image_transport
)

catkin_package()

catkin_install_python(PROGRAMS
  src/mission_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
