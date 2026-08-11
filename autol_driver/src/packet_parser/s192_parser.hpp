#ifndef S192_PCAP_PARSER_HPP
#define S192_PCAP_PARSER_HPP
#include "packet_parser/parser_manager.hpp"

int AutoLS192UdpPacket::packet_mode_ = 0;

class S192Parser : public Parser<AutoLS192UdpPacket>
{
public:
    S192Parser();
    virtual ~S192Parser() {}
    virtual void ChangePacketsToFov();
    
    bool ProcessPointsData(AutoLS192UdpPacket &packet, int lidar_num, unsigned long long& update_count, bool isFilePlay = false, bool isFistPacket = false, int packet_mode = 0);
	void ChangeFovToPcd(AutoLS192FovDataPointTmp &data_point, std::vector<DataPoint> &pcd_data);
private:
    uint32_t cur_frame, prev_frame;
    uint32_t debug_cur_frame, debug_prev_frame;
    vector<AutoLS192FovDataPointTmp> data_points;
    vector<DataPoint> pcd_data;
};

S192Parser::S192Parser()
{
    cur_frame = 0;
    prev_frame = 0;

    float angle = 90;
    int num_of_channel = 192;

	double vert_resolution = angle / (num_of_channel - 1);

	float angle_start = -angle / 2;

	for (size_t i = 0; i < num_of_channel; i++)
	{
		vertical_angle_arr_[i] = angle_start + vert_resolution * i;
	}

	data_points.reserve(192*256*2);
	pcd_data.reserve(192*256*2);
}

void S192Parser::ChangePacketsToFov()
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
		AutoLS192UdpPacket packet = { 0, };
        
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
        ProcessPointsData(packet, 0, update_count, false, false, packet.packet_mode_);

		// auto process_start = std::chrono::steady_clock::now();
		// ProcessPointsData(packet, 0, update_count, false, false, packet.packet_mode_);
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

bool S192Parser::ProcessPointsData(AutoLS192UdpPacket &packet, int lidar_num, unsigned long long& update_count, bool isFilePlay, bool isFistPacket, int packet_mode)
{
    auto ParsePacket = [&](int packet_mode)
	{
		if (packet_mode == 0)
		{
			S192PCDDataPakcet& pkt = packet.pcd_data_packet;
			const auto& hdr = pkt.common_header;
			const auto& ph = pkt.pixel_header;

			int total_az = hdr.total_azimuth_count;
			if (total_az > 256)
				total_az = 256;
			const int channel = ph.channel_index;
			const float dist_res = 1.0f / (float)ph.distance_resolution;

			// Intrinsic 적용
			static const double kCx = 124.0804996;
			static const double kCy = 97.43779592;
			static const double kF = 115.4421626;
			static const double kA = 0.399040;
			static const double kB = -0.110008;
			static const double kC = 0.279390;

			double yd = ((double)channel - kCy) / kF;
			double yd2 = yd * yd;
			int az_limit = total_az;


			for (int az = 0; az < az_limit; ++az)
			{
				AutoLS192FovDataPointTmp data_point;
				const auto& pd = pkt.points[az];
				if (pd.distance == 0) 
					continue;

				const double xd = ((double)az - kCx) / kF;
				const double r2 = xd * xd + yd2;
				const double g = 1.0 + kA * r2 + kB * r2 * r2 + kC * r2 * r2 * r2;
				const double rayX = xd * g;
				const double rayY = yd * g;
				const double norm = std::sqrt(rayX * rayX + rayY * rayY + 1.0);
				const double Z = ((float)pd.distance * dist_res) / norm;

				// Uncomment the necessary data
				// data_point.distance_m_ = (float)pd.distance * dist_res;
				data_point.reflectivity_ = pd.intensity;
				// data_point.azimuth_index_ = az;
				data_point.channel_index_ = channel;
				// data_point.echo_index_ = ph.echo_number;
				data_point.timestamp_ = hdr.timestamp;
				data_point.xPos_ = (float)(Z);
				data_point.yPos_ = (float)(-Z * rayX);
				data_point.zPos_ = (float)(Z * rayY);
				// data_point.vertical_angle_ = atan2(rayY, std::sqrt(1.0 + rayX * rayX)) * 180.0 / PI;			
				// data_point.azimuth_ = atan2(-rayX, 1.0) * 180.0 / PI;

				data_points.emplace_back(data_point);
				
				ChangeFovToPcd(data_point, pcd_data);
			}
		}
		else
		{
			S192XYZDataPakcet& pkt = packet.xyz_data_packet;
			const auto& hdr = pkt.common_header;
			const auto& ph = pkt.pixel_header;
			const int kXyzPointsPerPkt = 128;
			const float kXyzRadiusUnitM = (float)1 / 256;
			const float kXyzDirScale = 32768.0f;
			const int base_az = (int)ph.packet_number * kXyzPointsPerPkt;

			for (int i = 0; i < kXyzPointsPerPkt; ++i)
			{
				AutoLS192FovDataPointTmp data_point;
				const auto& pd = pkt.points[i];
				if (pd.radius == 0) 
					continue;

				const float r_m = pd.radius * kXyzRadiusUnitM;
				const float dx = pd.dir_x / kXyzDirScale;
				const float dy = pd.dir_y / kXyzDirScale;
				const float dz = pd.dir_z / kXyzDirScale;

				// Uncomment the necessary data
				//data_point.distance_m_ = r_m;
				data_point.reflectivity_ = pd.intensity;
				//data_point.azimuth_index_ = base_az + i;
				data_point.channel_index_ = ph.channel_index;
				//data_point.echo_index_ = ph.echo_number;
				data_point.timestamp_ = hdr.timestamp;
				data_point.xPos_ = r_m * dz;
				data_point.yPos_ = -r_m * dx;
				data_point.zPos_ = r_m * dy;
				//data_point.vertical_angle_ = asin(dy) * 180.0 / PI;
				//data_point.azimuth_ = atan2(-dx, dz) * 180.0 / PI;

				data_points.emplace_back(data_point);

				ChangeFovToPcd(data_point, pcd_data);
			}
		}
	};

	long lost_packet_cnt = 0;
	float motor_rpm = 0;
	bool isFrameCompleted = false;

	if(packet_mode ==0)
		cur_frame = packet.pcd_data_packet.pixel_header.frame_number;
	else
		cur_frame = packet.xyz_data_packet.pixel_header.frame_number;


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

            // publish the pcd data
            pcd_callback_(pcd_data, lidar_idx_);

			data_points.clear();
            pcd_data.clear();
			isFrameCompleted = true;

			// Not required for file playback, as data is output frame by frame.
			if (isFilePlay == false)
			{
				ParsePacket(packet_mode);				
			}

			fps++;
		}
	}
	else
	{
		ParsePacket(packet_mode);
	}
	prev_frame = cur_frame;

	return isFrameCompleted;
}

void S192Parser::ChangeFovToPcd(AutoLS192FovDataPointTmp& data_point, std::vector<DataPoint> &pcd_data)
{
	float pos_x = data_point.xPos_;
	float pos_y = data_point.yPos_;
	float pos_z = data_point.zPos_;
	float intensity = data_point.reflectivity_;
	int channelNum = data_point.channel_index_;
	uint64_t timestamp = data_point.timestamp_;

	//calibration
	if (lidar_config_.calibration == true)
	{
		ApplyRPY(pos_x, pos_y, pos_z, lidar_idx_, calibration_.lidar_slamoffset_corrections);
	}
	
	pcd_data.push_back({pos_x, pos_y, pos_z, intensity, channelNum, timestamp});
}

#endif