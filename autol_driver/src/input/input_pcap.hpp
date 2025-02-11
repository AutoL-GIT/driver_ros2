#ifndef INPUT_PCAP_HPP
#define INPUT_PCAP_HPP
#include "input_manager.hpp"
#include "lidar_controller/lidar_controller.hpp"
#include "packet_parser/parser_manager.hpp"
#include "packet_parser/g32_parser.hpp"
#include "packet_parser/g32_v2_parser.hpp"
#include "packet_parser/s56_parser.hpp"

class InputPcap : public InputManager
{
public:

    InputPcap(LIDAR_CONFIG &lidar_config, int32_t lidar_idx) : InputManager(lidar_config, lidar_idx)
    {
        packet_g32_callback_ = NULL;
        packet_g32_v2_callback_ = NULL;
        packet_s56_callback_ = NULL;
        pcd_callback_ = NULL;
    }
    virtual ~InputPcap() {}
    //Start to receive pcap data
    virtual void StartRecvData();
    //Stop to receive pcap data
    virtual void StopRecvData();
};

void InputPcap::StartRecvData()
{
    //lidar_ctrl_ptr_ = new G32Parser();
    //Assign parse function 
    switch (lidar_config_.model_id)
    {
    case ModelId::G32:
        if(lidar_config_.data_format_version == 1)
        {
            lidar_ctrl_ptr_ = new G32Parser();
            lidar_ctrl_ptr_->packet_g32_ctrl_callback_ = packet_g32_callback_;
            lidar_ctrl_ptr_->pcd_callback_ = pcd_callback_;
        }
        else if(lidar_config_.data_format_version == 2)
        {
            lidar_ctrl_ptr_ = new G32V2Parser();
            lidar_ctrl_ptr_->packet_g32_v2_ctrl_callback_ = packet_g32_v2_callback_;
            lidar_ctrl_ptr_->pcd_callback_ = pcd_callback_;
        }
        break;
    case ModelId::S56:
        lidar_ctrl_ptr_ = new S56Parser();
        lidar_ctrl_ptr_->packet_s56_ctrl_callback_ = packet_s56_callback_;
        lidar_ctrl_ptr_->pcd_callback_ = pcd_callback_;
        break;
    default:
        break;
    }
    //Start parser thread 
    lidar_ctrl_ptr_->StartParserThread(lidar_config_, lidar_idx_);
    //Get node address
    lidar_ctrl_ptr_->node_ = node_;
    //Get Lidar Configure address
    lidar_ctrl_ptr_->lidar_idx_ = lidar_idx_;
}

// Close Parser Thread
void InputPcap::StopRecvData()
{
    lidar_ctrl_ptr_->StopParserThread();
}

#endif