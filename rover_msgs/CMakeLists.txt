cmake_minimum_required(VERSION 3.0.2)
project(rover_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  WheelStates.msg
  PowerStatus.msg
  ArmJointStates.msg
  HazardParams.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
