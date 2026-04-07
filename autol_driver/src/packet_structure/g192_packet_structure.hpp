#ifndef G192_PACKET_STRUCTRUE_HPP
#define G192_PACKET_STRUCTRUE_HPP

//#include "define.hpp"

using namespace std;

 #pragma pack(push, 1)

	typedef struct
	{
		uint16_t ambient_;
	}AutoLG192Ambient; // 12 bytes

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
	}AutoLG192FovDataPointTmp; // 12 bytes


	typedef struct
	{
		float vertical_angle_;
		unsigned int distance_;
		unsigned short intensity_;
	}AutoLG192FovDataPoint; // 12 bytes

	typedef struct
	{
		float azimuth_;
		AutoLG192FovDataPoint data_points_[192];
	}AutoLG192FovDataBlock;

	typedef struct
	{
		union {
			uint8_t raw[24];
			struct {
				uint16_t magic_number;
				uint8_t proto_version_major;
				uint8_t proto_version_minor;
				uint16_t total_channel_count;
				uint16_t total_azimuth_count;
				uint8_t reserved[8];
				uint64_t timestamp;
			};
		};
	}CommonHeader;

	typedef struct
	{
		union {
			uint8_t raw[16];
			struct {
				uint8_t data_order; // 0: azimuth, 1: channel
				uint16_t data_order_number; // if data_order is 0 then 0~191, else 0~682
				uint8_t mirror_number; // 0~3
				uint8_t frame_number;
				uint16_t distance_resolution;
				uint8_t reserved[9];
			};
		};
	}DataHeader;

	typedef struct {
		union {
			uint8_t raw[3];
			struct {
				uint16_t distance;
				uint8_t intensity;
			};
		};
	}PCDData;
	typedef struct
	{
		union {
			uint8_t raw[576];
			struct {
				PCDData point[192];
			};
		};
	}PCDDataBlock;

	typedef struct
	{
		union {
			uint8_t raw[192];
			struct {
				uint8_t confidence[192];
			};
		};
	}PCDDataConfidence;

	typedef struct
	{
		union {
			uint8_t raw[40];
			struct {
				uint8_t reserved[40];
			};
		};
	}PCDDataTail;

	typedef struct
	{
		union {
			uint8_t raw[16];
			struct {
				uint8_t data_order; // 0: azimuth, 1: channel
				uint16_t data_oder_number; // if data_order is 0 then 0~191, else 0~682
				uint8_t mirror_number; // 0~3
				uint8_t frame_number;
				uint8_t reserved[11];
			};
		};
	}AmbientHeader;

	typedef struct
	{
		union {
			uint8_t raw[1152];
			struct {
				uint16_t ambient_data[576];
			};
		};
	}AmbientDataBlock;

	typedef struct
	{
		union {
			uint8_t raw[40];
			struct {
				uint8_t reserved[40];
			};
		};
	}AmbientDataTail;

	typedef struct
	{
		CommonHeader common_header;
		DataHeader data_header;
		PCDDataBlock echo[2];
		PCDDataConfidence data_confidence;
		PCDDataTail data_tail;
	}PCDDataPakcet;

	typedef struct
	{
		CommonHeader common_header;
		AmbientHeader ambient_header;
		AmbientDataBlock ambient;
		AmbientDataTail ambient_tail;
	}AmbientDataPakcet;

	class AutoLG192UdpPacket
	{
	public:
		PCDDataPakcet data_packet;
		AmbientDataPakcet ambient_data_packet;
		
		void DeSerializeUdpPacket(char* bytes, size_t size = 0)
		{
			if (size == 1232)
				memcpy(&ambient_data_packet, bytes, sizeof(AmbientDataPakcet));
			else
				memcpy(&data_packet, bytes, sizeof(PCDDataPakcet));
		}

		void AddDataBlockToFovDataSet(vector<AutoLG192FovDataBlock>& fov_data_set, float top_bottom_angle_offset, vector<int>& lidar_id_vector, float vertical_angle_arr_[], unsigned int& fov_data_arr_count_)
		{

		}
	};
 #pragma pack(pop)

#endif