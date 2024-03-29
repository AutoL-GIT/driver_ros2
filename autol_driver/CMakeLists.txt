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

project(autol_driver)

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
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_typesupport_c REQUIRED) 
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

pkg_search_module(LIBPCAP REQUIRED libpcap)
if(LIBPCAP_FOUND)
    message(STATUS "Found libpcap")
    include_directories(${LIBPCAP_INCLUDE_DIRS})
    link_directories(${LIBPCAP_LIBRARY_DIRS})
    add_definitions(${LIBPCAP_DEFINITIONS})
else()
    message(FATAL_ERROR "libpcap not found")
endif()
# ------------------------------------------------
# Source Code and Library
# ------------------------------------------------
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

    
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutolPacket.msg"
  "msg/AutolFrame.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)             

# ------------------------------------------------
# Driver Node Executable
# ------------------------------------------------
add_executable(${PROJECT_NAME}_node 
  src/driver/driver_node.cpp
)
# ------------------------------------------------
# Linking
# ------------------------------------------------

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${ament_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${PCAP_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  include
  src/driver
  src/input
  src/lidar_controller
  src/packet_parser
  src/packet_parser/g32_packet_parser
  src/packet_structure
  ${CMAKE_CURRENT_BINARY_DIR}
)


target_link_libraries(${PROJECT_NAME}_node
  ${LIBPCAP_LIBRARIES}
  ${RCLCPP_LIBRARIES}
  ${SENSOR_MSGS_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
# ------------------------------------------------
# Installation
# ------------------------------------------------
ament_export_include_directories(include)

find_package(ament_index_cpp REQUIRED)
ament_target_dependencies(${PROJECT_NAME}_node 
  ament_index_cpp
  rclcpp
  rcl_interfaces
  std_msgs
  sensor_msgs
  pcl_conversions
)
rosidl_target_interfaces(${PROJECT_NAME}_node  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY launch rviz include/ 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

