#ifndef G32_PCAP_PARSER_HPP
#define G32_PCAP_PARSER_HPP
#include "packet_parser/parser_manager.hpp"

class G32Parser : public Parser<AutoLG32UdpPacket>
{
public:
    G32Parser();
    virtual ~G32Parser() {}
    virtual void ChangePacketsToFov();
    virtual void ChangeFovToPcd(std::vector<AutoLG32FovDataBlock> &fov_data_set_t, std::vector<DataPoint> &pcd_data);
    virtual void SetVerticalAngle(ModelId model_id, float angle);

private:
    float vertical_angle_arr_[32];
    int data_points_size_;
    float top_bottom_offset;
};

G32Parser::G32Parser()
{
    data_points_size_ = 16;
    int angle = 10;
    int num_of_channel = 16;

    top_bottom_offset = angle / (2 * num_of_channel);

    float angle_start = -angle / 2 + top_bottom_offset / 2;
    //Set the vertical angle offset
    for (int32_t i = 0; i < num_of_channel; i++)
    {
        vertical_angle_arr_[i] = angle_start + (angle / num_of_channel) * i;
        vertical_angle_arr_[i + 16] = angle_start + (angle / num_of_channel) * i + top_bottom_offset;
    }
}

void G32Parser::ChangePacketsToFov()
{
    long long cur_packet_id = 0;
    long long prev_packet_id = 0;
    long long num_of_lost_packet = 0;
    bool is_first_fov_data = true;
    int stage_count = 0;
    unsigned long long update_count = 0;
    vector<AutoLG32FovDataBlock> fov_data_set_t;
    vector<AutoLG32UdpPacket> frame_data;
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
        AutoLG32UdpPacket packet;
        queue_mutex.lock();
        //Get packet data from packet_queue
        if (!packet_queue.empty())
        {
            packet = packet_queue.front();
            packet_queue.pop();
        }
        else
        {
            queue_mutex.unlock();
            usleep(20);
            continue;
        }
        queue_mutex.unlock();

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
                    SetVerticalAngle(ModelId::G32, vert_angle);
                    if (!(fov_data_set_t.size() == 0 && packet.header_.top_bottom_side_ == 1))
                    {
                        packet.AddDataBlockToFovDataSet(fov_data_set_t, top_bottom_offset, lidar_id_vector_, vertical_angle_arr_, fov_data_arr_count_);
                        frame_data.emplace_back(packet);
                        stage_count++;
                    }
                    if (stage_count >= 2)
                    {
                        update_count++;
                        bool is_fov_ok = true;
                        for (size_t i = 0; i < fov_data_set_t.size() / 2; i++)
                        {
                            if (fov_data_set_t[i].data_points_->vertical_angle_ == fov_data_set_t[i + fov_data_set_t.size() / 2].data_points_->vertical_angle_)
                                is_fov_ok = false;
                        }
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
                            packet_callback_(frame_data, lidar_idx_);
                            // packet to pcd 
                            ChangeFovToPcd(fov_data_set_t, pcd_data);
                            // publish the pcd data
                            pcd_callback_(pcd_data, lidar_idx_);
                            // re-init
                            pcd_data.clear();
                            fov_data_set_t.clear();
                            frame_data.clear();
                            frame_data.emplace_back(packet);
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
                    }
                }
            }

            if (!(packet.header_.data_type_ == 0xA5B3C2AA))
            {
                SetVerticalAngle(ModelId::G32, vert_angle);
                frame_data.emplace_back(packet);
                packet.AddDataBlockToFovDataSet(fov_data_set_t, top_bottom_offset, lidar_id_vector_, vertical_angle_arr_, fov_data_arr_count_);
            }
        }
    }
}

void G32Parser::SetVerticalAngle(ModelId device_id, float angle)
{
    switch (device_id)
    {
    case ModelId::G32:
    {
        int num_of_channel = 16;
        top_bottom_offset = angle / (2 * num_of_channel);
        float angle_start = -angle / 2 + top_bottom_offset / 2;

        for (int32_t i = 0; i < num_of_channel; i++)
        {
            vertical_angle_arr_[i] = angle_start + (angle / num_of_channel) * i;
            vertical_angle_arr_[i + 16] = angle_start + (angle / num_of_channel) * i + top_bottom_offset;
        }
    }
    break;
    default:
        break;
    }
}

void G32Parser::ChangeFovToPcd(std::vector<AutoLG32FovDataBlock> &fov_data_set_t, std::vector<DataPoint> &pcd_data)
{
    const int32_t numOfChannel = 16;
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
                                   fov_data_set_t[i].azimuth_, pos_x, pos_y, pos_z, 0);
            // calibration
            if (lidar_config_.calibration == true)
            {
                ApplyRPY(pos_x, pos_y, pos_z, lidar_idx_, calibration_.lidar_slamoffset_corrections);
            }
            pcd_data.push_back({pos_x, pos_y, pos_z, intensity, (uint16_t)j, timestamp});
        }
    }
}

#endif