#ifndef G32_PACKET_STRUCTRUE_HPP
#define G32_PACKET_STRUCTRUE_HPP

#include "define.hpp"

using namespace std;
#define BUF_SIZE pow(2, 20) * 1024
#define PACKET_DATA_SIZE 1330
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
} AutoLG32FovDataPoint; // 12 bytes

typedef struct
{
	float azimuth_;
	AutoLG32FovDataPoint data_points_[32];
} AutoLG32FovDataBlock;

#pragma pack(push, 1)
typedef struct
{
	unsigned short tof_; // 2 byte
	uint8_t intensity_;	 // 1 byte
} ChannelData;			 // total : 3 byte
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	uint16_t flag_; // 2 byte
	int azimuth_;	// 4 byte
	// channel_t		Channel_Data[2][16];
	ChannelData channel_data_[16]; // 3 * 16 = 48 byte
} DataBlock;					   // Total : 54 byte
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	uint32_t motor_rpm_;
	uint32_t voltage_data_;
	uint32_t voltage_fraction_;
	uint8_t frame_rate;		// 7 byte
	uint8_t vertical_angle; // 7 byte
	char reserved_[5];		// 23 byte
} EsTestInfo;

typedef struct
{
	uint8_t frame_rate;		// 7 byte
	uint8_t vertical_angle; // 7 byte
	char reserved[17];		// 7 byte
} LidarInfo;

typedef struct
{
	int packet_id_;			  // 4 byte
	uint8_t top_bottom_side_; // 1 byte
	uint32_t data_type_;

	union
	{
		EsTestInfo es_test_info_;
		LidarInfo lidar_info_;
	};
} Header; // total : 28 byte
#pragma pack(pop)

class AutoLG32UdpPacket
{
public:
	Header header_;			   // 28 byte
	DataBlock data_block_[24]; // 1296 byte
	uint32_t time_;			   // 4 byte
	uint16_t factory_;		   // 2 byteTimestamp

	void DeSerializeUdpPacket(char *bytes, size_t size)
	{
		const int32_t g32PacketSize = 1330;
		memcpy(this, bytes, g32PacketSize);
	}

	void AddDataBlockToFovDataSet(vector<AutoLG32FovDataBlock> &fov_data_set, float top_bottom_angle_offset, vector<int> &lidar_id_vector,
								  float vertical_angle_arr_[], unsigned int &fov_data_arr_count_)
	{
		// cout << _countof(data_block_) << endl;
		AutoLG32FovDataBlock fov_data_block[24];
		if (header_.top_bottom_side_ == 0)
		{
			for (size_t i = 0; i < _countof(data_block_); i++)
			{

				fov_data_block[i].azimuth_ = (float)data_block_[i].azimuth_ / 1000;
				for (size_t j = 0; j < _countof(DataBlock::channel_data_); j++)
				{

					fov_data_block[i].data_points_[j].azimuth_ = 0;
					fov_data_block[i].data_points_[j].distance_ = (unsigned int)data_block_[i].channel_data_[j].tof_;
					fov_data_block[i].data_points_[j].reflectivity_ = data_block_[i].channel_data_[j].intensity_;
					fov_data_block[i].data_points_[j].vertical_angle_ = vertical_angle_arr_[j];
				}
				fov_data_set.emplace_back(fov_data_block[i]);
				// if (lidar_id == 0x11)
				lidar_id_vector.emplace_back(1);
				/*		else if (lidar_id == 0x12)
							lidar_id_vector.emplace_back(2);*/
				fov_data_arr_count_++;
			}
			// cout << "1" << endl;
		}
		//}
		// else if(stage_count == 1)
		//{
		if (header_.top_bottom_side_ == 1)
		{
			for (size_t i = 0; i < _countof(data_block_); i++)
			{

				fov_data_block[i].azimuth_ = (float)data_block_[i].azimuth_ / 1000;
				for (size_t j = 0; j < _countof(DataBlock::channel_data_); j++)
				{
					fov_data_block[i].data_points_[j].azimuth_ = 0;
					fov_data_block[i].data_points_[j].distance_ = (unsigned int)data_block_[i].channel_data_[j].tof_;
					fov_data_block[i].data_points_[j].reflectivity_ = data_block_[i].channel_data_[j].intensity_;
					fov_data_block[i].data_points_[j].vertical_angle_ = vertical_angle_arr_[j] + top_bottom_angle_offset;

					// if (fov_data_set.size() == 477 && j == 6)
					//{
					//	cout << fov_data_block[i].data_points_[j].distance_ << endl;
					// }
				}
				fov_data_set.emplace_back(fov_data_block[i]);
				/*if (lidar_id == 0x11)*/
				lidar_id_vector.emplace_back(1);
				/*	else if (lidar_id == 0x12)
						lidar_id_vector.emplace_back(2);*/
				fov_data_arr_count_++;
			}
			// cout << "2" << endl;
		}
	}
};

#pragma pack(pop)

#endif