# ------------------------------------------------
# Project Setup
# ------------------------------------------------
# Specify minimum CMake version required for this project
cmake_minimum_required(VERSION 3.8)
# ------------------------------------------------
# Set default C and C++ language standards if not already defined
# ------------------------------------------------
# Default to C99
if(NOT CMAKE_C_STANDARD)
set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()


# Include a script to determine the ROS version being used
# Define the project name, version, and languages
project(autol_example LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ------------------------------------------------
# Find Dependencies
# ------------------------------------------------
# Find required ROS2 packages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# ------------------------------------------------
# Package Metadata
# ------------------------------------------------
# Configure package information for ROS2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# ------------------------------------------------
# Source Code and Library
# ------------------------------------------------
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# Create the autol_driver library
add_library(${PROJECT_NAME}_lib SHARED
  src/RcvPcd.cpp
)

# ------------------------------------------------
# Point Cloud Node Executable
# ------------------------------------------------
add_executable(${PROJECT_NAME}_node 
  src/example_node.cpp
)
# ------------------------------------------------
# Linking
# ------------------------------------------------
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${RCLCPP_LIBRARIES}
  ${SENSOR_MSGS_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# ------------------------------------------------
# Installation
# ------------------------------------------------

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(
  ${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  sensor_msgs
)

ament_package()
