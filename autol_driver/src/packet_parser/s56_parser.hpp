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
    vector<DataPoint> pcd_data;
    vector<AutoLS56UdpPacket> frame_data;

    long long cur_packet_id = 0;
	long long prev_packet_id = 0;
	long long num_of_lost_packet = 0;
	bool is_packet_lost = false;
	bool is_first_fov_data = true;
	int stage_count = 0;
	long lost_packet_cnt = 0;
	unsigned long long update_count = 0;
	unsigned long long motor_rpm = 0;
	unsigned long long voltage_data = 0;
	unsigned long long voltage_fraction = 0;
	int32_t ground_z_axis_value = 0;
	//NetLogger::GetInstance()->logger_->critical("{}", "PacketsToFov Start : " + to_string(thread_id));

	vector<AutoLS56FovDataBlock> fov_data_set_t(192 * 3); 


	for (size_t i = 0; i < fov_data_set_t.size(); i++)
	{
		fov_data_set_t[i].azimuth_ = 60 - 0.625 * (i / 3 );
		for (size_t j = 0; j < 56; j++)
		{
			fov_data_set_t[i].data_points_[j].distance_ = 0.0;
			fov_data_set_t[i].data_points_[j].reflectivity_ = 0.0;
		}
	}

	vector<timeval> time_stamp_vector;
	time_stamp_vector.reserve(10000);
	vector<int> lidar_id_vector_;
    
	
	start_vec = std::chrono::system_clock::now();
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
		AutoLS56UdpPacket packet = { 0, };

        unique_lock<mutex> lock(mtx);
        cv.wait(lock, [this]{return !packet_queue.empty();});
        while(!packet_queue.empty())
        {
             packet = packet_queue.front();
             packet_queue.pop();

            cur_packet_id = packet.header_.packet_number;
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
                    is_packet_lost = true;
                }
            }

            prev_packet_id = cur_packet_id;

            if (packet.header_.packet_number == 167)
            {
                update_count++;
                if (is_first_fov_data)
                {
                    stage_count = 0;

                    fov_data_arr_count_ = 0;
                    //fov_data_set_t.clear();
                    //time_stamp_vector.clear();
                    lidar_id_vector_.clear();
                    fov_data_arr_count_ = 0;
                    is_first_fov_data = false;
                    last_lost_packet = 0;
                }
                else
                {
                    SetVerticalAngle(ModelId::S56, 14.7321428571429 * 2);

                    packet.AddDataBlockToFovDataSet(fov_data_set_t, top_bottom_offset, lidar_id_vector_, vertical_angle_arr_, fov_data_arr_count_);
                    frame_data.emplace_back(packet);


                    //publish the packet data
                    //packet_s56_ctrl_callback_(frame_data, lidar_idx_);
                    // packet to pcd 
                    ChangeFovToPcd(fov_data_set_t, pcd_data);
                    // publish the pcd data
                    pcd_callback_(pcd_data, lidar_idx_);

                    // re-init
                    pcd_data.clear();
                    //fov_data_set_t.clear();
                    frame_data.clear();


                    fps++;
                    if (update_count == std::numeric_limits<uint64_t>::max())
                        update_count = 1;

                    is_packet_lost = false;
                    lidar_id_vector_.clear();
                    fov_data_arr_count_ = 0;
                }
            }
            else
            {
                SetVerticalAngle(ModelId::S56, 14.7321428571429 * 2);
                packet.AddDataBlockToFovDataSet(fov_data_set_t, top_bottom_offset, lidar_id_vector_, vertical_angle_arr_, fov_data_arr_count_);
                frame_data.emplace_back(packet);
            }
        }
	}

	if (stop_udp_thread_ == true)
	{
		fps = 0;
		last_fps = 0;
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
                                   fov_data_set_t[i].azimuth_, pos_x, pos_y, pos_z, 0, 128);
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