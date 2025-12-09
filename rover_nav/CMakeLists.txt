cmake_minimum_required(VERSION 3.0.2)
project(rover_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  rover_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/depth_hazard_monitor.py
  src/hazard_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
