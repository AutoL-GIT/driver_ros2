#ifndef S56_PACKET_STRUCTRUE_HPP
#define S56_PACKET_STRUCTRUE_HPP

#include "define.hpp"



#pragma once
#include <stdint.h>
#include <vector>

using namespace std;

#pragma pack(push, 1)

typedef struct
{
	float vertical_angle_;
	unsigned int distance_;
	float azimuth_;
	float ambient_;
	union
	{
		uint8_t reflectivity_;
		unsigned short echo_pulse_width_;
	};
}AutoLS56FovDataPoint; // 12 bytes

typedef struct
{
	float azimuth_;
	AutoLS56FovDataPoint data_points_[56];
}AutoLS56FovDataBlock;

#pragma pack(push, 1)
typedef struct
{
	unsigned short	tof_;		// 2 byte
	short		intensity_;		// 1 byte
}ChannelDataAutoLS56; 						// total : 3 byte
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	uint16_t		flag_;					// 2 byte
	int				azimuth_;				// 4 byte
	//channel_t		Channel_Data[2][16];	
	ChannelDataAutoLS56		channel_data_[56];		// 3 * 16 = 48 byte
}DataBlockAutoLS56;									// Total : 54 byte
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	uint32_t	motor_rpm_;
	uint32_t	voltage_data_;
	uint32_t	voltage_fraction_;
	uint8_t		frame_rate;				// 7 byte
	uint8_t		vertical_angle;				// 7 byte
	char		reserved_[5];		// 23 byte
}EsTestInfoAutoLS56;

typedef struct
{
	uint8_t frame_rate;				// 7 byte
	uint8_t vertical_angle;				// 7 byte
	int32_t ground_z_axis_value;
	char reserved[13];				// 7 byte
}LidarInfoAutoLS56;

typedef struct
{
	int 		packet_id_; 		// 4 byte
	uint8_t		top_bottom_side_;   // 1 byte
	uint32_t	data_type_;

	union {
		EsTestInfoAutoLS56 es_test_info_;
		LidarInfoAutoLS56 lidar_info_;
	};
}HeaderAutoLS56;							// total : 28 byte
#pragma pack(pop)

class AutoLS56UdpPacket
{
public:
	HeaderAutoLS56		header_;					// 28 byte
	DataBlockAutoLS56	data_block_[4];			// 1296 byte
	uint32_t	time_;						// 4 byte
	uint16_t	factory_;					// 2 byteTimestamp

	void DeSerializeUdpPacket(char* bytes, size_t size = 1330)
	{
		memcpy(this, bytes, sizeof(AutoLS56UdpPacket));
	}

	void AddDataBlockToFovDataSet(vector<AutoLS56FovDataBlock>& fov_data_set, float top_bottom_angle_offset, vector<int>& lidar_id_vector, float vertical_angle_arr_[], unsigned int& fov_data_arr_count_)
	{
		AutoLS56FovDataBlock fov_data_block[24];

		if (header_.top_bottom_side_ == 0)
			for (size_t i = 0; i < _countof(data_block_); i++)
			{
				fov_data_block[i].azimuth_ = (float)data_block_[i].azimuth_ / 1000;

				for (size_t j = 0; j < _countof(DataBlockAutoLS56::channel_data_); j++)
				{
					fov_data_block[i].data_points_[j].distance_ = (unsigned int)data_block_[i].channel_data_[j].tof_;
					fov_data_block[i].data_points_[j].echo_pulse_width_ = data_block_[i].channel_data_[j].intensity_;
					fov_data_block[i].data_points_[j].vertical_angle_ = vertical_angle_arr_[j];
				}
				fov_data_set.emplace_back(fov_data_block[i]);
				lidar_id_vector.emplace_back(1);

				fov_data_arr_count_++;
			}
		if (header_.top_bottom_side_ == 1)
			for (size_t i = 0; i < _countof(data_block_); i++)
			{
				if ((float)data_block_[i].azimuth_ == 0)
					continue;

				fov_data_block[i].azimuth_ = (float)data_block_[i].azimuth_ / 1000;
				for (size_t j = 0; j < _countof(DataBlockAutoLS56::channel_data_); j++)
				{
					fov_data_block[i].data_points_[j].distance_ = (unsigned int)data_block_[i].channel_data_[j].tof_;
					fov_data_block[i].data_points_[j].echo_pulse_width_ = data_block_[i].channel_data_[j].intensity_;
					fov_data_block[i].data_points_[j].vertical_angle_ = vertical_angle_arr_[j] + top_bottom_angle_offset;
				}
				fov_data_set.emplace_back(fov_data_block[i]);

				lidar_id_vector.emplace_back(1);

				fov_data_arr_count_++;
			}
	}
};
#pragma pack(pop)

#endif