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
project(autol_pcd_saver)

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
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# ------------------------------------------------
# Point Cloud Node Executable
# ------------------------------------------------
add_executable(${PROJECT_NAME}_node 
  src/pcd_saver_node.cpp
)
# ------------------------------------------------
# Linking
# ------------------------------------------------
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${RCLCPP_LIBRARIES}
  ${SENSOR_MSGS_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ------------------------------------------------
# Installation
# ------------------------------------------------
ament_export_include_directories(include)

find_package(ament_index_cpp REQUIRED)
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
)

install(TARGETS ${PROJECT_NAME}_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
