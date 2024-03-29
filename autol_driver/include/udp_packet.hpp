#ifndef AUTOL_ROS_UDP_PACKET_HPP_
#define AUTOL_ROS_UDP_PACKET_HPP_
#include "define.hpp"

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

#endif

