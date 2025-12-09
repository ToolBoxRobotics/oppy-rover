cmake_minimum_required(VERSION 3.0.2)

project(rover_description)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro  
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  rover_msgs

)

catkin_package(
  CATKIN_DEPENDS
  urdf
  xacro
  roscpp

)


include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS src/six_wheel_kinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)