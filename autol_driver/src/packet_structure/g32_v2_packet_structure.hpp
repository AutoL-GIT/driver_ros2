#ifndef G32_v2_PACKET_STRUCTRUE_HPP
#define G32_v2_PACKET_STRUCTRUE_HPP

#include "define.hpp"

using namespace std;
#define BUF_SIZE pow(2, 20) * 1024
#define _countof(_Array)     sizeof(_Array) / sizeof(_Array[0])
#define PI 3.14159265358979323846

#pragma pack(push, 1)

typedef struct
{
	float vertical_angle_;
	unsigned int distance_;
	float azimuth_;
	union
	{
		uint8_t reflectivity_;
		unsigned short echo_pulse_width_;
	};
} AutoLG32V2FovDataPoint; // 12 bytes

typedef struct
{
	float azimuth_;
	AutoLG32V2FovDataPoint data_points_[32];
} AutoLG32V2FovDataBlock;

#pragma pack(push, 1)
typedef struct
{
	unsigned short tof_; // 2 byte
	unsigned short intensity_;	 // 1 byte
} AutoLG32V2ChannelData;			 // total : 3 byte
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	uint16_t flag_; // 2 byte
	int azimuth_;	// 4 byte
	// channel_t		Channel_Data[2][16];
	AutoLG32V2ChannelData channel_data_[16]; // 3 * 16 = 48 byte
} AutoLG32V2DataBlock;					   // Total : 54 byte
#pragma pack(pop)

class AutoLG32V2UdpPacket
{
public:
	Header header_;			   // 28 byte
	AutoLG32V2DataBlock data_block_[20]; // 1296 byte
	uint32_t time_;			   // 4 byte
	uint16_t factory_;		   // 2 byteTimestamp

	void DeSerializeUdpPacket(char *bytes, size_t size = 1330)
	{
		const int32_t g32PacketSize = 1434;
		memcpy(this, bytes, g32PacketSize);
	}

	void AddDataBlockToFovDataSet(vector<AutoLG32V2FovDataBlock> &fov_data_set, float top_bottom_angle_offset, vector<int> &lidar_id_vector,
								  float vertical_angle_arr_[], unsigned int &fov_data_arr_count_)
	{
		AutoLG32V2FovDataBlock fov_data_block[20];
		if (header_.top_bottom_side_ == 0)
		{
			for (size_t i = 0; i < _countof(data_block_); i++)
			{
				fov_data_block[i].azimuth_ = (float)data_block_[i].azimuth_ / 1000;
				for (size_t j = 0; j < _countof(AutoLG32V2DataBlock::channel_data_); j++)
				{

					fov_data_block[i].data_points_[j].azimuth_ = 0;
					fov_data_block[i].data_points_[j].distance_ = (unsigned int)data_block_[i].channel_data_[j].tof_;
					fov_data_block[i].data_points_[j].echo_pulse_width_ = data_block_[i].channel_data_[j].intensity_ & 0x03FF;
					fov_data_block[i].data_points_[j].vertical_angle_ = vertical_angle_arr_[j];
				}
				fov_data_set.emplace_back(fov_data_block[i]);
				lidar_id_vector.emplace_back(1);
				fov_data_arr_count_++;
			}
		}
		if (header_.top_bottom_side_ == 1)
		{
			for (size_t i = 0; i < _countof(data_block_); i++)
			{

				fov_data_block[i].azimuth_ = (float)data_block_[i].azimuth_ / 1000;
				for (size_t j = 0; j < _countof(AutoLG32V2DataBlock::channel_data_); j++)
				{
					fov_data_block[i].data_points_[j].azimuth_ = 0;
					fov_data_block[i].data_points_[j].distance_ = (unsigned int)data_block_[i].channel_data_[j].tof_;
					fov_data_block[i].data_points_[j].echo_pulse_width_ = data_block_[i].channel_data_[j].intensity_ & 0x03FF;
					fov_data_block[i].data_points_[j].vertical_angle_ = vertical_angle_arr_[j] + top_bottom_angle_offset;
				}
				fov_data_set.emplace_back(fov_data_block[i]);
				lidar_id_vector.emplace_back(1);
				fov_data_arr_count_++;
			}
		}
	}
};

#pragma pack(pop)

#endif