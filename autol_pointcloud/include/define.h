#ifndef AUTOL_ROS_DEFINE_H_
#define AUTOL_ROS_DEFINE_H_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <autol_msgs/msg/autol_packet.hpp>
#include <autol_msgs/msg/autol_frame.hpp>

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cstdarg>
#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>

static const int DATABLOCK_SIZE = 24;
static const int CHANNEL_SIZE = 16;
static const float VERTICAL_FOV = 10;

#endif // AUTOL_ROS_DEFINE_INClUDE_H_