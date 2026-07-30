#ifndef S56_PACKET_STRUCTRUE_HPP
#define S56_PACKET_STRUCTRUE_HPP

#include "define.hpp"
#include "g192_packet_structure.hpp"
#pragma once
#include <stdint.h>
#include <vector>

using namespace std;

#pragma pack(push, 1)

typedef struct
{
	uint16_t ambient_;
}AutoLS56Ambient; // 12 bytes

typedef struct
{
	float azimuth_;
	float azimuth_raw_;
	float vertical_angle_;
	float distance_;
	unsigned short intensity_;
	unsigned short ambient_;
	int mirror_number;
	int channel_number_;
}AutoLS56FovDataPointTmp; // 12 bytes


typedef struct
{
	float vertical_angle_;
	unsigned int distance_;
	unsigned short intensity_;
}AutoLS56FovDataPoint; // 12 bytes

typedef struct
{
	float azimuth_;
	AutoLS56FovDataPoint data_points_[192];
}AutoLS56FovDataBlock;


class AutoLS56UdpPacket
{
public:
	PCDDataPakcet data_packet;
	AmbientDataPakcet ambient_data_packet;

	void DeSerializeUdpPacket(char* bytes, size_t size)
	{
		if (size == 1232)
			memcpy(&ambient_data_packet, bytes, sizeof(AmbientDataPakcet));
		else if (size == 1424)
			memcpy(&data_packet, bytes, sizeof(PCDDataPakcet));
	}

	void AddDataBlockToFovDataSet(vector<AutoLS56FovDataBlock>& fov_data_set, float top_bottom_angle_offset, vector<int>& lidar_id_vector, float vertical_angle_arr_[], unsigned int& fov_data_arr_count_)
	{
	}
};
#pragma pack(pop)

#endif