#ifndef S56_PCAP_PARSER_HPP
#define S56_PCAP_PARSER_HPP
#include "packet_parser/parser_manager.hpp"

#include <climits>

class S56Parser : public Parser<AutoLS56UdpPacket>
{
public:
    S56Parser();
    virtual ~S56Parser() {}
    virtual void ChangePacketsToFov();
    
    bool ProcessPointsData(AutoLS56UdpPacket &packet, int lidar_num, unsigned long long& update_count, bool isFilePlay = false, bool isFistPacket = false);
	void ChangeFovToPcd(AutoLS56FovDataPointTmp &data_point, std::vector<DataPoint> &pcd_data);
private:
	static const int kAzimuthCount = 192;
	static const int kChannelCount = 56;
	static const int kEchoCount = 2;
	static const int kAmbientAzimuthCount = 576;

    uint32_t cur_frame, prev_frame;
    uint32_t debug_cur_frame, debug_prev_frame;    	
	array<array<array<AutoLS56FovDataPointTmp, kEchoCount>, kChannelCount>, kAzimuthCount> data_points{};
	array<array<AutoLS56Ambient, kChannelCount>, kAmbientAzimuthCount> data_ambient;
    vector<DataPoint> pcd_data;
};

S56Parser::S56Parser()
{
    cur_frame = 0;
    prev_frame = 0;

    float angle = 34.375;
    int num_of_channel = 56;

	double vert_resolution = angle / (num_of_channel - 1);

	float angle_start = -angle / 2;

	for (size_t i = 0; i < num_of_channel; i++)
	{
		vertical_angle_arr_[i] = angle_start + vert_resolution * i;
	}
}


void S56Parser::ChangePacketsToFov()
{        
    long lost_packet_cnt = 0;
	unsigned long long update_count = 0;
	float motor_rpm = 0;

    start_vec = std::chrono::system_clock::now();
	while (stop_packets2fov_thread_ == false)
	{
		// Get Packet
        end_vec = std::chrono::system_clock::now();
        if ((chrono::duration_cast<chrono::microseconds>(end_vec - start_vec)).count() >= 1000000)
		{
            start_vec = std::chrono::system_clock::now();
			last_fps = fps;
			fps = 0;
			last_lost_packet = lost_packet;
		}
		AutoLS56UdpPacket packet = { 0, };
        
		{
			std::unique_lock<std::mutex> lock(queue_mutex);
			cv.wait(lock, [this]()
			{
                return stop_packets2fov_thread_ || !packet_queue.empty();
			});

			if (stop_packets2fov_thread_ && packet_queue.empty())
			{
			    break;
            }

			if(packet_queue.size() > 0)
			{
				packet = std::move(packet_queue.front());
				packet_queue.pop();
			}
		}
        ProcessPointsData(packet, 0, update_count, false, false);

        // auto process_start = std::chrono::steady_clock::now();
        // ProcessPointsData(packet, 0, update_count, false, false);
        // auto process_end = std::chrono::steady_clock::now();
        // auto process_us =std::chrono::duration_cast<std::chrono::microseconds>(process_end - process_start).count();
        // std::cerr << "ProcessPointsData : "<< static_cast<long long>(process_us) << " " << packet_queue.size() << std::endl;
	}

	if (stop_packets2fov_thread_ == true)
	{
		fps = 0;
		last_fps = 0;
	}
}

bool S56Parser::ProcessPointsData(AutoLS56UdpPacket &packet, int lidar_num, unsigned long long& update_count, bool isFilePlay, bool isFistPacket)
{
    auto ParsePacket = [&]()
    {
        for (size_t pixel_ind = 0; pixel_ind < packet.data_packet.common_header.total_azimuth_count; pixel_ind++)
		{
			AutoLS56FovDataPointTmp data_point;
			data_point.azimuth_ = 59.6875 - pixel_ind * 0.625;  
			data_point.azimuth_raw_ = 59.6875 - pixel_ind * 0.625; 
            data_point.channel_number_ = packet.data_packet.data_header.data_order_number;
            data_point.vertical_angle_ = vertical_angle_arr_[packet.data_packet.data_header.data_order_number];

            for (size_t echo_ind = 0; echo_ind < _countof(packet.data_packet.echo); echo_ind++)
            {
                if((float)packet.data_packet.echo[echo_ind].point[pixel_ind].distance == 0)
                    continue;

                data_point.distance_ = (float)packet.data_packet.echo[echo_ind].point[pixel_ind].distance / packet.data_packet.data_header.distance_resolution;
                data_point.intensity_ = packet.data_packet.echo[echo_ind].point[pixel_ind].intensity;
                data_point.mirror_number = packet.data_packet.data_header.mirror_number - 1;
                if (pixel_ind < data_points.size()
                    && packet.data_packet.data_header.data_order_number < data_points.at(pixel_ind).size()
                    && echo_ind < data_points.at(pixel_ind).at(packet.data_packet.data_header.data_order_number).size())
                    data_points.at(pixel_ind).at(packet.data_packet.data_header.data_order_number).at(echo_ind) = data_point;


                    ChangeFovToPcd(data_point, pcd_data);
            }
		}
    };

	long lost_packet_cnt = 0;
	float motor_rpm = 0;
	bool isFrameCompleted = false;

	cur_frame = packet.data_packet.data_header.frame_number;


	if (isFilePlay && isFistPacket)
		prev_frame = cur_frame;

	if (cur_frame != prev_frame)
	{
		size_t data_point_size = data_points.size();
		if (data_point_size > 0)
		{
			lidar_num++;
			if (lidar_num > 1)
				lidar_num = 1;


            // publish the pcd data
            pcd_callback_(pcd_data, lidar_idx_);

			if (update_count == ULLONG_MAX)
				update_count = 1;


            data_points = {};
            pcd_data.clear();
			isFrameCompleted = true;

			// Not required for file playback, as data is output frame by frame.
			if (isFilePlay == false)
			{
				ParsePacket();				
			}

			fps++;
		}
	}
	else
	{
		ParsePacket();
	}
	prev_frame = cur_frame;

	return isFrameCompleted;
}

void S56Parser::ChangeFovToPcd(AutoLS56FovDataPointTmp& data_point, std::vector<DataPoint> &pcd_data)
{
    if(data_point.distance_ > 0)
    {
        float intensity = 0;
        int channelNum = 0;
        uint64_t timestamp = 0;
        
        float pos_x = 0;
        float pos_y = 0;
        float pos_z = 0;
        intensity = data_point.intensity_;
        channelNum = data_point.channel_number_;
        ConvertPolorToOrthCood((float)data_point.distance_, data_point.vertical_angle_, data_point.azimuth_, pos_x, pos_y, pos_z, 0, 1.0);

        // calibration
        if (lidar_config_.calibration == true)
        {
            ApplyRPY(pos_x, pos_y, pos_z, lidar_idx_, calibration_.lidar_slamoffset_corrections);
        }
        pcd_data.push_back({pos_x, pos_y, pos_z, intensity, channelNum, timestamp});
    }
    
}

#endif