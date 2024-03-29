#ifndef LIDAR_CONTROLLER_HPP
#define LIDAR_CONTROLLER_HPP

#include "define.hpp"
#include "utils.hpp"
#include "packet_structure/g32_packet_structure.hpp"

class LidarController
{
public:
    virtual ~LidarController() {}
    virtual void StartParserThread(LIDAR_CONFIG& lidar_config, int32_t lidarIdx) = 0;
    virtual void StopParserThread() = 0;
    virtual void ChangePacketsToFov() = 0;
    virtual void ChangeFovToPcd(std::vector<AutoLG32FovDataBlock> &fov_data_set_t, std::vector<DataPoint> &pcd_data) = 0;
    SendFrameCallback packet_callback_;
    SendPcdCallback pcd_callback_;
    rclcpp::Node *node_;
    int32_t lidar_idx_;
};
#endif