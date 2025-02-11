#ifndef S56_PCAP_PARSER_HPP
#define S56_PCAP_PARSER_HPP
#include "packet_parser/parser_manager.hpp"

class S56Parser : public Parser<AutoLS56UdpPacket>
{
public:
    S56Parser();
    virtual ~S56Parser() {}
    virtual void ChangePacketsToFov();
    
    void ChangeFovToPcd(std::vector<AutoLS56FovDataBlock> &fov_data_set_t, std::vector<DataPoint> &pcd_data);
private:
};

S56Parser::S56Parser()
{
	data_points_size_ = 56;
	for (size_t i = 0; i < data_points_size_; i++)
	{
		vertical_angle_arr_[i] = -15 + ((float)30 / (56 - 1)) * i;
	}
}

void S56Parser::ChangePacketsToFov()
{
    long long cur_packet_id = 0;
    long long prev_packet_id = 0;
    long long num_of_lost_packet = 0;
    bool is_first_fov_data = true;
    int stage_count = 0;
    unsigned long long update_count = 0;
    vector<AutoLS56FovDataBlock> fov_data_set_t;
    vector<AutoLS56UdpPacket> frame_data;
    vector<DataPoint> pcd_data;
    vector<int> lidar_id_vector_;
    start_vec = std::chrono::system_clock::now();
    //Change packet to Point Cloud
    while (stop_packets2fov_thread_ == false)
    {
        this_thread::yield();

        end_vec = std::chrono::system_clock::now();
        if ((chrono::duration_cast<chrono::microseconds>(end_vec - start_vec)).count() >= 1000000)
        {
            start_vec = std::chrono::system_clock::now();
            last_fps = fps;
            fps = 0;

            last_lost_packet = lost_packet;
        }
        AutoLS56UdpPacket packet;
        unique_lock<mutex> lock(mtx);;

        cv.wait(lock, [this]{return !packet_queue.empty();});
        while(!packet_queue.empty())
        {
            packet = packet_queue.front();
            packet_queue.pop();
            
            if (packet.header_.data_type_ != 0)
            {
                //Check the loss packet   
                cur_packet_id = packet.header_.packet_id_;
                if (cur_packet_id != (prev_packet_id + 1))
                {
                    if (cur_packet_id != 0)
                    {
                        if (cur_packet_id - prev_packet_id > 0)
                        {
                            num_of_lost_packet += cur_packet_id - prev_packet_id;
                            lost_packet += cur_packet_id - prev_packet_id;
                        }
                        else
                        {
                            lost_packet += (cur_packet_id - prev_packet_id) + 288;
                            num_of_lost_packet += cur_packet_id - prev_packet_id + 288;
                        }
                    }
                }

                prev_packet_id = cur_packet_id;

                if (packet.header_.data_type_ == 0xA5B3C2AA && packet.header_.packet_id_ != 0)
                {
                    
                    //init configuration
                    if (is_first_fov_data)
                    {
                        stage_count = 0;

                        fov_data_arr_count_ = 0;
                        fov_data_set_t.clear();
                        lidar_id_vector_.clear();
                        fov_data_arr_count_ = 0;
                        is_first_fov_data = false;
                        last_lost_packet = 0;
                    }
                    else
                    {
                        SetVerticalAngle(ModelId::S56, vert_angle);
                        //if (!(fov_data_set_t.size() == 0 && packet.header_.top_bottom_side_ == 1))
                        {
                            packet.AddDataBlockToFovDataSet(fov_data_set_t, top_bottom_offset, lidar_id_vector_, vertical_angle_arr_, fov_data_arr_count_);
                            frame_data.emplace_back(packet);
                            stage_count++;
                        }
                        //if (stage_count >= 2)
                        {                        
                            update_count++;
                            bool is_fov_ok = true;
                            
                            if (is_fov_ok)
                            {
                                if (packet.header_.lidar_info_.frame_rate <= 50)
                                {
                                    frame_rate = packet.header_.lidar_info_.frame_rate;
                                    vertical_angle = packet.header_.lidar_info_.vertical_angle;
                                }
                                else
                                {
                                    frame_rate = packet.header_.es_test_info_.frame_rate;
                                    vertical_angle = packet.header_.es_test_info_.vertical_angle;
                                }
                                //publish the packet data
                                //packet_s56_ctrl_callback_(frame_data, lidar_idx_);
                                // packet to pcd 
                                ChangeFovToPcd(fov_data_set_t, pcd_data);
                                // publish the pcd data
                                pcd_callback_(pcd_data, lidar_idx_);
                                
                                // re-init
                                pcd_data.clear();
                                fov_data_set_t.clear();
                                frame_data.clear();
                            }
                            fps++;
                            if (update_count == ULLONG_MAX)
                            {
                                update_count = 1;
                            }
                            lidar_id_vector_.clear();
                            stage_count = 0;
                            fov_data_arr_count_ = 0;
                            fov_data_set_t.clear();
                            frame_data.clear();
                        }
                    }
                }

                if (!(packet.header_.data_type_ == 0xA5B3C2AA))
                {
                    SetVerticalAngle(ModelId::S56, vert_angle);
                    frame_data.emplace_back(packet);
                    packet.AddDataBlockToFovDataSet(fov_data_set_t, top_bottom_offset, lidar_id_vector_, vertical_angle_arr_, fov_data_arr_count_);
                }
            }
        }
    }
}

void S56Parser::ChangeFovToPcd(std::vector<AutoLS56FovDataBlock> &fov_data_set_t, std::vector<DataPoint> &pcd_data)
{
    const int32_t numOfChannel = 56;
    float intensity = 0;
    double timestamp = 0;

    for (int32_t i = 0; i < (int32_t)fov_data_set_t.size(); i++)
    {
        for (int32_t j = 0; j < numOfChannel; j++)
        {
            float pos_x = 0;
            float pos_y = 0;
            float pos_z = 0;
            intensity = fov_data_set_t[i].data_points_[j].reflectivity_;
            ConvertPolorToOrthCood((float)fov_data_set_t[i].data_points_[j].distance_,
                                   fov_data_set_t[i].data_points_[j].vertical_angle_,
                                   fov_data_set_t[i].azimuth_, pos_x, pos_y, pos_z, 0, 256/0.3);
            // calibration
            if (lidar_config_.calibration == true)
            {
                ApplyRPY(pos_x, pos_y, pos_z, lidar_idx_, calibration_.lidar_slamoffset_corrections);
            }
            if(fov_data_set_t[i].data_points_[j].distance_ > 0)
                pcd_data.push_back({pos_x, pos_y, pos_z, intensity, (uint16_t)j, timestamp});
        }
    }
}
#endif