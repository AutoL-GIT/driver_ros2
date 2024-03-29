#ifndef INPUT_SOCKET_HPP
#define INPUT_SOCKET_HPP

#include "define.hpp"
#include "input_manager.hpp"
#include "lidar_controller/lidar_controller.hpp"
#include "packet_parser/g32_packet_parser/g32_parser.hpp"
#define PACKET_DATA_SIZE 1330
#define UDP_PORT 5001

class InputSocket : public InputManager
{

public:
    InputSocket(LIDAR_CONFIG &lidar_config, int32_t lidar_idx): InputManager(lidar_config, lidar_idx)
    {
        packet_callback_ = NULL;
        pcd_callback_ = NULL;
    }
    virtual ~InputSocket(){}
    virtual void StartRecvData();
    virtual void StopRecvData();

};


void InputSocket::StartRecvData()
{
    switch (lidar_config_.model_id)
    {
    case ModelId::G32:
        lidar_ctrl_ptr_ = new G32Parser();
        lidar_ctrl_ptr_->packet_callback_ = packet_callback_;
        lidar_ctrl_ptr_->pcd_callback_= pcd_callback_;
        break;
    default:

        break;
    }

    lidar_ctrl_ptr_->StartParserThread(lidar_config_, lidar_idx_);
    lidar_ctrl_ptr_->node_ = node_;
    lidar_ctrl_ptr_->lidar_idx_ = lidar_idx_;
}

void InputSocket::StopRecvData()
{
    lidar_ctrl_ptr_->StopParserThread();
}
#endif