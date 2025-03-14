#ifndef AUTOL_ROS_DEFINE_HPP_
#define AUTOL_ROS_DEFINE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <signal.h>
#include <stdint.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/socket.h>
#include <boost/shared_ptr.hpp>
#include <std_msgs/msg/string.hpp>
#include <netinet/in.h>
#include <pcap.h>
#include <vector>
#include <queue>
#include <fstream>

#include "autol_driver/msg/autol_g32_packet.hpp"
#include "autol_driver/msg/autol_g32_frame.hpp"
#include "autol_driver/msg/autol_g32_v2_packet.hpp"
#include "autol_driver/msg/autol_g32_v2_frame.hpp"
#include "autol_driver/msg/autol_s56_packet.hpp"
#include "autol_driver/msg/autol_s56_frame.hpp"

#include "driver.hpp"
#include "packet_structure/g32_packet_structure.hpp"
#include "packet_structure/g32_v2_packet_structure.hpp"
#include "packet_structure/s56_packet_structure.hpp"

#define UNUSED(x) (void)(x)
#define MAX_NUM_LIDAR 6
enum class ManufatureId
{
    autol
};

enum class ModelId
{
    G32,
    G192, 
    S56
};
enum class InputType
{
    UDP,
    PCAP,
    TCP
};

typedef struct
{
    float_t x;
    float_t y;
    float_t z;
    float_t intensity;
    uint16_t ring;
    double timestamp;
} DataPoint;

typedef struct
{
    ManufatureId manufacture_id;
    ModelId model_id;
    int32_t data_format_version;
    InputType input_type;
    int32_t lidar_count;
    int32_t packet_per_frame;
    int32_t frame_rate;
    int32_t read_once;
    int32_t read_fast;
    bool calibration;
    std::string pcap_path;
    std::vector<int32_t> lidar_port;

} LIDAR_CONFIG;

static const int DATABLOCK_SIZE = 24;
static const int CHANNEL_SIZE = 16;
static const float VERTICAL_FOV = 10;


typedef std::vector<DataPoint> PointData;
typedef std::vector<AutoLG32UdpPacket> G32FrameData_t;
typedef std::vector<AutoLG32V2UdpPacket> G32V2FrameData_t;
typedef std::vector<AutoLS56UdpPacket> S56FrameData_t;
typedef std::function<void(const G32FrameData_t &, int32_t)> SendFrameG32Callback;
typedef std::function<void(const G32V2FrameData_t &, int32_t)> SendFrameG32V2Callback;
typedef std::function<void(const S56FrameData_t &, int32_t)> SendFrameS56Callback;
typedef std::function<void(const PointData &, int32_t)> SendPcdCallback;

#endif // AUTOL_ROS_DEFINE_INClUDE_H_