#ifndef G192_PCAP_PARSER_HPP
#define G192_PCAP_PARSER_HPP
#include "packet_parser/parser_manager.hpp"

class G192Parser : public Parser<AutoLG192UdpPacket>
{
public:
    G192Parser();
    virtual ~G192Parser() {}
    virtual void ChangePacketsToFov();
    
    bool ProcessPointsData(AutoLG192UdpPacket &packet, int lidar_num, unsigned long long& update_count, bool isFilePlay = false, bool isFistPacket = false);
    void ChangeFovToPcd(std::vector<AutoLG192FovDataPointTmp> &fov_data_set_t, std::vector<DataPoint> &pcd_data);
    void ApplyCalFiles();
private:
    uint32_t cur_frame, prev_frame;
    uint32_t debug_cur_frame, debug_prev_frame;
    vector<AutoLG192FovDataPointTmp> data_points;
    vector<DataPoint> pcd_data;
};

G192Parser::G192Parser()
{
    cur_frame = 0;
    prev_frame = 0;

    float angle = 25;
    int num_of_channel = 192;

	double vert_resolution = angle / (num_of_channel - 1);

	float angle_start = -angle / 2;

	for (size_t i = 0; i < num_of_channel; i++)
	{
		vertical_angle_arr_[i] = angle_start + vert_resolution * i;
	}
    ApplyCalFiles();
}
void G192Parser::ApplyCalFiles()
{

}

void G192Parser::ChangePacketsToFov()
{        
    long lost_packet_cnt = 0;
	unsigned long long update_count = 0;
	float motor_rpm = 0;

    start_vec = std::chrono::system_clock::now();
	while (stop_udp_thread_ == false)
	{
		// Get Packet
		this_thread::yield();
        end_vec = std::chrono::system_clock::now();
        if ((chrono::duration_cast<chrono::microseconds>(end_vec - start_vec)).count() >= 1000000)
		{
            start_vec = std::chrono::system_clock::now();
			last_fps = fps;
			fps = 0;
			last_lost_packet = lost_packet;
		}
		AutoLG192UdpPacket packet = { 0, };
        
		queue_mutex.lock();
		if (!packet_queue.empty())
		{
			packet = packet_queue.front();
			packet_queue.pop();
		}
		else
		{
			queue_mutex.unlock();
            usleep(1);
			continue;
		}
		queue_mutex.unlock();
        ProcessPointsData(packet, 0, update_count);        
	}

	if (stop_udp_thread_ == true)
	{
		fps = 0;
		last_fps = 0;
	}
}

bool G192Parser::ProcessPointsData(AutoLG192UdpPacket &packet, int lidar_num, unsigned long long& update_count, bool isFilePlay, bool isFistPacket)
{
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
			//update_count++;
			lidar_num++;
			if (lidar_num > 1)
				lidar_num = 1;

			// if (update_count == MAXULONGLONG)
			// 	update_count = 1;

            ChangeFovToPcd(data_points, pcd_data);
            // publish the pcd data
            pcd_callback_(pcd_data, lidar_idx_);


			data_points.clear();
            pcd_data.clear();
			 
			isFrameCompleted = true;

			// 파일 재생일 경우, 프레임 단위 출력이기 때문에 아래 코드 불필요
			if (isFilePlay == false)
			{
				double azimuth_resolution = 120.0 / (packet.data_packet.common_header.total_azimuth_count - 1);
				azimuth_resolution = (std::round(azimuth_resolution * 1000)) / 1000.0;
				double start_azimuth = azimuth_resolution * (packet.data_packet.common_header.total_azimuth_count - 1) / 2;
				for (size_t pixel_ind = 0; pixel_ind < packet.data_packet.common_header.total_channel_count; pixel_ind++)
				{
					AutoLG192FovDataPointTmp data_point;
					data_point.azimuth_ = start_azimuth - packet.data_packet.data_header.data_order_number * azimuth_resolution;
					data_point.azimuth_raw_ = start_azimuth - packet.data_packet.data_header.data_order_number * azimuth_resolution;
					data_point.channel_number_ = pixel_ind;
					data_point.vertical_angle_ = vertical_angle_arr_[pixel_ind];

					for (size_t echo_ind = 0; echo_ind < _countof(packet.data_packet.echo); echo_ind++)
					{
						data_point.distance_ = (float)packet.data_packet.echo[echo_ind].point[pixel_ind].distance / packet.data_packet.data_header.distance_resolution;
						data_point.intensity_ = packet.data_packet.echo[echo_ind].point[pixel_ind].intensity;
						data_point.mirror_number = packet.data_packet.data_header.mirror_number - 1;
						data_points.emplace_back(data_point);
					}
				}
			}

			fps++;
		}
	}
	else
	{
		double azimuth_resolution = 120.0 / (packet.data_packet.common_header.total_azimuth_count - 1);
		azimuth_resolution = (std::round(azimuth_resolution * 1000)) / 1000.0;
		double start_azimuth = azimuth_resolution * (packet.data_packet.common_header.total_azimuth_count - 1) / 2;
		for (size_t pixel_ind = 0; pixel_ind < packet.data_packet.common_header.total_channel_count; pixel_ind++)
		{
			AutoLG192FovDataPointTmp data_point;
			data_point.azimuth_ = start_azimuth - packet.data_packet.data_header.data_order_number * azimuth_resolution;
			data_point.azimuth_raw_ = start_azimuth - packet.data_packet.data_header.data_order_number * azimuth_resolution;
			{
				{
					data_point.channel_number_ = pixel_ind;
					data_point.vertical_angle_ = vertical_angle_arr_[pixel_ind];
				}
				for (size_t echo_ind = 0; echo_ind < _countof(packet.data_packet.echo); echo_ind++)
				{
					data_point.distance_ = (float)packet.data_packet.echo[echo_ind].point[pixel_ind].distance / packet.data_packet.data_header.distance_resolution;
					data_point.intensity_ = packet.data_packet.echo[echo_ind].point[pixel_ind].intensity;
					data_point.mirror_number = packet.data_packet.data_header.mirror_number - 1;
					data_points.emplace_back(data_point);
				}
			}
		}
	}
	prev_frame = cur_frame;

	return isFrameCompleted;
}

void G192Parser::ChangeFovToPcd(std::vector<AutoLG192FovDataPointTmp> &data_points, std::vector<DataPoint> &pcd_data)
{
    const int32_t numOfChannel = 16;
    float intensity = 0;
    int channelNum = 0;
    double timestamp = 0;

    for (int32_t i = 0; i < (int32_t)data_points.size(); i++)
    {
         if(data_points[i].distance_ > 0)
         {
            int azimuthIdx = i / (192 * 2);
            int channelIdx = (i % (192 * 2) / 2);

            double azimthOffset = calibration_.M_H2[data_points[i].mirror_number][channelIdx][azimuthIdx];	
            double channelOffset = calibration_.M_V2[data_points[i].mirror_number][channelIdx][azimuthIdx];	

            float pos_x = 0;
            float pos_y = 0;
            float pos_z = 0;
            intensity = data_points[i].intensity_;
            channelNum = data_points[i].channel_number_;
            ConvertPolorToOrthCood((float)data_points[i].distance_,
                                    data_points[i].vertical_angle_ + channelOffset,
                                    data_points[i].azimuth_ + azimthOffset, pos_x, pos_y, pos_z, 0, 1.0);
            // calibration
            if (lidar_config_.calibration == true)
            {
                ApplyRPY(pos_x, pos_y, pos_z, lidar_idx_, calibration_.lidar_slamoffset_corrections);
            }
            
            pcd_data.push_back({pos_x, pos_y, pos_z, intensity, channelNum, timestamp});
        }
    }
}
#endif