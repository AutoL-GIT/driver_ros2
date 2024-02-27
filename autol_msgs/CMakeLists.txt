# ------------------------------------------------
# Project Setup
# ------------------------------------------------
# Specify minimum CMake version required for this project
cmake_minimum_required(VERSION 3.8)
# ------------------------------------------------
# Set default C and C++ language standards if not already defined
# ------------------------------------------------
project(autol_msgs)
# ------------------------------------------------
# ROS1-Specific Configuration (Conditional)
# ------------------------------------------------
if(ROS_EDITION STREQUAL "ROS1")
  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(
    DIRECTORY msg
    FILES 
    AutolPacket.msg 
    AutolFrame.msg
  )

  generate_messages(DEPENDENCIES std_msgs)

  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES autol_msgs
    CATKIN_DEPENDS message_runtime std_msgs
  #  DEPENDS system_lib
  )

  include_directories(
  # include
    ${catkin_INCLUDE_DIRS}
  )
# ------------------------------------------------
# ROS2-Specific Configuration (Conditional)
# ------------------------------------------------
else(ROS_EDITION STREQUAL "ROS2")
  # ------------------------------------------------
  # Find required ROS2 packages
  # ------------------------------------------------
  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  #-------------------------------------------------
  # Message Generation (Conditional)
  # ------------------------------------------------
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AutolPacket.msg"
    "msg/AutolFrame.msg"
    DEPENDENCIES builtin_interfaces std_msgs
  )
  ament_export_dependencies(rosidl_default_runtime)

  include_directories(
    # include
    ${ament_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
  )
  
  ament_package()
endif()
