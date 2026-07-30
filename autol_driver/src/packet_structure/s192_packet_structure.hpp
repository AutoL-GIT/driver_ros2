#ifndef S192_PACKET_STRUCTRUE_HPP
#define S192_PACKET_STRUCTRUE_HPP

using namespace std;

#pragma pack(push, 1)

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
            uint8_t reserved[6];
            uint8_t packet_mode;
            uint8_t time_source;
            uint64_t timestamp;
        };
    };
}S192CommonHeader;

typedef struct
{
    union {
        uint8_t raw[16];
        struct {
            uint8_t scan_direction;
            uint16_t channel_index;
            uint8_t reserved0;
            uint8_t frame_number;
            uint16_t distance_resolution;
            uint8_t echo_number;
            uint8_t reserved1[8];
        };
    };
}S192PCDPixelHeader;

typedef struct
{
    union {
        uint8_t raw[16];
        struct {
            uint8_t scan_direction;
            uint16_t channel_index;
            uint8_t reserved0;
            uint8_t frame_number;
            uint16_t distance_resolution;
            uint8_t echo_number;
            uint8_t packet_number;
            uint8_t reserved1[7];
        };
    };
}S192XYZPixelHeader;

typedef struct {
    union {
        uint8_t raw[4];
        struct {
            uint16_t distance;
            uint8_t intensity;
            uint8_t flags;
        };
    };
}S192PCDData;

typedef struct {
    union {
        uint8_t raw[10];
        struct {
            uint16_t radius;
            int16_t dir_x;
            int16_t dir_y;
            int16_t dir_z;
            uint8_t intensity;
            uint8_t flags;
        };
    };
}S192XYZData;

typedef struct
{
    union {
        uint8_t raw[40];
        struct {
            uint8_t reserved[40];
        };
    };
}S192PCDDataTail;


typedef struct
{
    S192CommonHeader common_header;
    S192PCDPixelHeader pixel_header;
    S192PCDData points[256];
    S192PCDDataTail data_tail;
}S192PCDDataPakcet;

typedef struct
{
    S192CommonHeader common_header;
    S192XYZPixelHeader pixel_header;
    S192XYZData points[128];
    S192PCDDataTail data_tail;
}S192XYZDataPakcet;

typedef struct
{
    uint16_t ambient_;
}AutoLS192Ambient; // 12 bytes

typedef struct
{
    float vertical_angle_;
    unsigned int distance_;
    float distance_m_;
    float azimuth_;
    float ambient_;
    int channel_index_;
    int azimuth_index_;
    int echo_index_;
    uint64_t timestamp_;
    float xPos_;
    float yPos_;
    float zPos_;
    union
    {
        uint8_t reflectivity_;
        unsigned short echo_pulse_width_;
    };
}AutoLS192FovDataPointTmp; // 12 bytes

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
}AutoLS192FovDataPoint; // 12 bytes

typedef struct
{
    float azimuth_;
    AutoLS192FovDataPoint data_points_[192];
}AutoLS192FovDataBlock;

class AutoLS192UdpPacket
{
public:
    S192PCDDataPakcet pcd_data_packet;
    S192XYZDataPakcet xyz_data_packet;
    static int packet_mode_;

    void DeSerializeUdpPacket(char* bytes, size_t size = 1104)
    {
        try
        {			
            if (size == 1104)
            {
                packet_mode_ = 0;
                memcpy(&pcd_data_packet, bytes, sizeof(S192PCDDataPakcet));
            }
            else if (size == 1360)
            {
                packet_mode_ = 1;
                memcpy(&xyz_data_packet, bytes, sizeof(S192XYZDataPakcet));
            }
        }
        catch (const std::exception& e)
        {
            std::cout << "memcpy exception: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cout << "unknown memcpy exception" << std::endl;
        }
    }

    void AddDataBlockToFovDataSet(vector<AutoLS192FovDataBlock>& fov_data_set, float top_bottom_angle_offset, vector<int>& lidar_id_vector, float vertical_angle_arr_[], unsigned int& fov_data_arr_count_)
    {
    }
};
#pragma pack(pop)

#endif