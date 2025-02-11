#ifndef LIDAR_CONTROLLER_HPP
#define LIDAR_CONTROLLER_HPP

#include "define.hpp"
#include "utils.hpp"

class LidarController
{
public:
    virtual ~LidarController() {}

    //Packet Input Thread
    virtual void StartParserThread(LIDAR_CONFIG& lidar_config, int32_t lidarIdx) = 0;
    virtual void StopParserThread() = 0;

    //Packet Parse Thread: Pakcet ->  Azimuth, Elevation
    virtual void ChangePacketsToFov() = 0;

    //Callback Constructor Variable 
    SendFrameG32Callback packet_g32_ctrl_callback_;
    SendFrameG32V2Callback packet_g32_v2_ctrl_callback_;
    SendFrameS56Callback packet_s56_ctrl_callback_;
    SendPcdCallback pcd_callback_;
    rclcpp::Node *node_;
    int32_t lidar_idx_;


};
#endif