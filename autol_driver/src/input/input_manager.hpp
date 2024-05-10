#ifndef INPUT_MANAGER_HPP
#define INPUT_MANAGER_HPP
#include "define.hpp"
#include "packet_structure/g32_packet_structure.hpp"
#include "lidar_controller/lidar_controller.hpp"
class InputManager
{
protected:
    rclcpp::Node *node_;
    LIDAR_CONFIG lidar_config_;
    int32_t lidar_idx_;
    // Callback Function 
    std::function<void(const G32FrameData_t&, int32_t)> packet_callback_;
    std::function<void(const G32PointData&, int32_t)> pcd_callback_;
    //lidar controller class
    LidarController *lidar_ctrl_ptr_;
public:
    typedef std::shared_ptr<InputManager> Ptr;
    //Start to receive packet data through pcap or socket
    virtual void StartRecvData() = 0;
    //Stop to receive packet data
    virtual void StopRecvData() = 0;
    //Get Ros node
    void SetRosNode(rclcpp::Node *node){ node_ = node;}
    //Assign packet data callback function
    void RegRecvCallback(const std::function<void(const G32FrameData_t&, int32_t)>& callback);
    //Assign pcd data callback function
    void RegRecvPcdCallback(const std::function<void(const G32PointData&, int32_t)>& callback);
    InputManager(LIDAR_CONFIG &lidar_config, int32_t lidar_idx)
    {
        lidar_config_ = lidar_config;
        lidar_idx_ = lidar_idx;
    }
    virtual ~InputManager()
    {
        ;
    }

};

//Packet Callback
void InputManager::RegRecvCallback(const std::function<void(const G32FrameData_t&, int32_t)>& callback)
{
    packet_callback_ = callback;
}

//Point Cloud Callback
void InputManager::RegRecvPcdCallback(const std::function<void(const G32PointData&, int32_t)>& callback)
{
    pcd_callback_ = callback;
}
#endif