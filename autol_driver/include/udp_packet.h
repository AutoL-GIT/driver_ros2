#ifndef AUTOL_ROS_UDP_PACKET_H_
#define AUTOL_ROS_UDP_PACKET_H_
#include "define.h"

using namespace std;

typedef struct ip_address {
	u_char byte1;
	u_char byte2;
	u_char byte3;
	u_char byte4;
}ip_address;  // 4byte

typedef struct udp_header {
	u_short sport;          // Source port
	u_short dport;          // Destination port
	u_short len;            // Datagram length
	u_short crc;            // Checksum
}udp_header;  // 8byte

#pragma pack(push, 2)
typedef struct ethernet_header {
	char destination[6];
	char source[6];
	u_short type;
}ethernet_header; // 14 byte
#pragma pack(pop)

/* IPv4 header */
typedef struct ip_header {
	u_char  ver_ihl;        // Version (4 bits) + Internet header length (4 bits)
	u_char  tos;            // Type of service 
	u_short tlen;           // Total length 
	u_short identification; // Identification
	u_short flags_fo;       // Flags (3 bits) + Fragment offset (13 bits)
	u_char  ttl;            // Time to live
	u_char  proto;          // Protocol
	u_short crc;            // Header checksum
	ip_address  saddr;      // Source address
	ip_address  daddr;      // Destination address
	//u_int   op_pad;         // Option + Padding
}ip_header; // 20 byte

typedef struct autol_header {
	ethernet_header e_hdr;
	ip_header ip_hdr;
	udp_header udp_hdr;
}autol_header;

/// <summary>
/// ////////////////////////////////////////////////////////////////////////////
/// </summary>

#pragma pack(push, 1)
typedef struct
{
	unsigned short	tof_;		// 2 byte
	uint8_t		intensity_;		// 1 byte
}ChannelData; 						// total : 3 byte
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	uint16_t		flag_;					// 2 byte
	int				azimuth_;				// 4 byte
	//channel_t		Channel_Data[2][16];	
	ChannelData			channel_data_[16];		// 3 * 16 = 48 byte
}DataBlock;									// Total : 54 byte
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	int 		packet_id_; 		// 4 byte
	uint8_t		top_bottom_side_;   // 1 byte
	uint32_t	data_type_;
	char		reserved_[19];		// 23 byte

}Header;							// total : 28 byte
#pragma pack(pop)

#pragma pack(push, 2)
typedef struct
{
	Header		header_;					// 28 byte
	DataBlock	data_block_[24];			// 1296 byte
	uint32_t	time_;						// 4 byte
	uint16_t	factory_;					// 2 byteTimestamp
}UdpPacket;
#pragma pack(pop)

///////////////////////// Valeo /////////////////////////

typedef struct
{
	char time_stamp[8];
	uint8_t protocol_version;
	uint8_t magic_number;
	uint16_t sequence_number;
	uint8_t flags;
	uint8_t scanner_id;
	uint16_t data_type_id;
	uint16_t software_version;
	uint16_t scan_number;
	uint16_t fragments_total;
	uint16_t fragment_number;
}SUTPHeader;

#pragma pack(push, 1)
typedef struct 
{
	uint32_t stream_type;
	uint32_t time_stamp;
	uint32_t frame_size;
	uint8_t device_id;
}StreamTypeHeader;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	uint32_t magic_word;
	uint32_t time_stamp;
	uint32_t frame_size;
	uint16_t data_type;
}DebugDataHeader;
#pragma pack(pop)

typedef struct
{
	union
	{
		StreamTypeHeader stream_type_header;
		char payload_buffer[1448];
	};
}PayLoad;

typedef struct
{
	SUTPHeader sutp_header;
	PayLoad payload;
}ValeoUdpPacket;



///////////////////////// Velodyne /////////////////////////
#pragma pack(push, 1)
typedef struct
{
	unsigned short distance_;
	uint8_t reflectivity_;
} VelodyneDataPoint;
#pragma pack(pop)

typedef struct
{
	char flag_[2];
	unsigned short azimuth_;
	VelodyneDataPoint data_points_[32];
}VelodyneDataBlock;

#pragma pack(push, 2)
typedef struct
{
	VelodyneDataBlock data_block_[12];
	unsigned int time_stamp_;
	char return_mode_;
	char product_model_;
}VelodyneUdpPacket;
#pragma pack(pop)

//TCP TEST STURCT//
#pragma pack(push, 1)
typedef struct
{
	int 		Prefix;		// 4
	uint32_t	Body_length;	// 4
}header_t;


typedef struct
{
	uint8_t		Instr;			// 1
	int 		Ch;
	int			Value;
	uint8_t		aa;
}in_body;

typedef struct
{
	header_t		Header;			// 1
	in_body 		Body;
}In_Packet;
#endif

