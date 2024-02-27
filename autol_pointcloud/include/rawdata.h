#ifndef AUTOL_ROS_RAW_DATA_H_
#define AUTOL_ROS_RAW_DATA_H_

#include "define.h"
#include "convertdatabase.h"
#include "calibration.h"
#include <filesystem>

namespace autol_data
{
#pragma pack(push, 1)
    typedef struct
    {
        unsigned short tof_; // 2 byte
        uint8_t intensity_;  // 1 byte
    } ChannelData;           // total : 3 byte
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct
    {
        uint16_t flag_;                // 2 byte
        int azimuth_;                  // 4 byte
        ChannelData channel_data_[16]; // 3 * 16 = 48 byte
    } DataBlock;                       // Total : 54 byte
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct
    {
        int packet_id_;           // 4 byte
        uint8_t top_bottom_side_; // 1 byte
        uint32_t data_type_;
        char reserved_[19]; // 23 byte
    } Header;               // total : 28 byte
#pragma pack(pop)

#pragma pack(push, 2)
    typedef struct
    {
        Header header_;            // 28 byte
        DataBlock data_block_[24]; // 1296 byte
        uint32_t time_;            // 4 byte
        uint16_t factory_;         // 2 byteTimestamp
    } UdpPacket;
#pragma pack(pop)

#define PI 3.14159265358979323846

    enum DeviceID
    {
        AUTOL_G32
    };

    class RawData
    {
    public:
        RawData(rclcpp::Node *node);
        ~RawData() {}

        void ConvertFromRaw(const autol_msgs::msg::AutolPacket &packets, autol_data::ConvertDataBase &convert_data, bool is_slam_active, int lidar_index);
        int pointsPerPacket();
        void Setup();

    private:
        void DeSerialize(UdpPacket *udp_packet, char *bytes);
        void SetVerticalAngle(DeviceID device_id, int num_of_channel, float angle);
        void Get3DCoordinates(float distance, float elevation, float azimuth_offset, float z_axes_offset, float &pos_x, float &pos_y, float &pos_z);
        void ApplyRPY(float &pos_x, float &pos_y, float &pos_z, int lidar_id, std::vector<autol_pointcloud::SlamOffset> &rpy);

        float vertical_angle_arr_[128];
        float top_bottom_offset;

        autol_pointcloud::Calibration calibration_;
        std::vector<autol_pointcloud::SlamOffset> rpy_;
        rclcpp::Node *node_;
    };
}
#endif
