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
	int16_t distance;
	int16_t intensity;
	int16_t reserved;
}S56DataBlock;									// Total : 54 byte
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
	uint16_t magic_number;
	uint16_t protocol_version;
	int32_t reserved;
	uint16_t total_channel_number;
	uint16_t total_azimuth_number;
	uint8_t total_echo_number;
	uint8_t frame_period;
	uint16_t reserved2;
	uint8_t return_mode;
	uint8_t echo_number;
	uint16_t frame_number;
	uint16_t packet_number;
	uint16_t distance_resolution;
	uint8_t channel_azimuth_mode;
	uint8_t channel_azimuth_number;
	uint8_t utc_time[6];
	uint32_t time_stamp;
	int16_t vertical_resolution;
	int16_t horizontal_resolution;
}S56Header;							// total : 28 byte
#pragma pack(pop)

class AutoLS56UdpPacket
{
public:
	S56Header		header_;					// 28 byte
	S56DataBlock	data_blocks_[192];			// 1296 byte
	uint8_t factory;
	uint8_t reserved[47];

	void DeSerializeUdpPacket(char* bytes, size_t size = 1330)
	{
		const int32_t PacketSize = 1240;
		memcpy(this, bytes, PacketSize);
	}

	void AddDataBlockToFovDataSet(vector<AutoLS56FovDataBlock>& fov_data_set, float top_bottom_angle_offset, vector<int>& lidar_id_vector, float vertical_angle_arr_[], unsigned int& fov_data_arr_count_)
	{

		AutoLS56FovDataBlock fov_data_block[24];

		for (size_t i = 0; i < _countof(data_blocks_); i++)
		{
			
				fov_data_set[i * 3 + header_.echo_number].data_points_[header_.channel_azimuth_number].distance_ = (unsigned int)data_blocks_[i].distance;
				fov_data_set[i * 3 + header_.echo_number].data_points_[header_.channel_azimuth_number].echo_pulse_width_ = data_blocks_[i].intensity;
				fov_data_set[i * 3 + header_.echo_number].data_points_[header_.channel_azimuth_number].vertical_angle_ = vertical_angle_arr_[header_.channel_azimuth_number];
			
			
			lidar_id_vector.emplace_back(1);
			if(header_.channel_azimuth_number == 55 && header_.echo_number == 2 && i == 191)
				fov_data_arr_count_++;

		}
	}
};
#pragma pack(pop)

#endif