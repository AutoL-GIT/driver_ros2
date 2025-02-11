#ifndef PARSER_MANGER_HPP_
#define PARSER_MANGER_HPP_

#include "define.hpp"
#include "utils.hpp"
#include "udp_socket.hpp"
#include "input/input_manager.hpp"
#include "lidar_controller/lidar_controller.hpp"
#include <pthread.h>
#include <condition_variable>

template <class LidarUdpPacket>
class Parser : public LidarController
{
public:
    Parser() {}
    virtual ~Parser() {}
    //Packet ReceiveThread and ChangeToFov Thread Management Function
    virtual void StartParserThread(LIDAR_CONFIG &lidar_config, int32_t idx);
    virtual void StopParserThread();
    //Packet Receive Thread
    void ReceiveThreadDowork();
    //Make FramePackage and Convert pcd
    //void ChangePacketsToFovSyn(AutoLG32UdpPacket lidar_udp_packet, int32_t idx);
    //Socket Connect Function
    bool ConnectSocket(int port_num);
    //Packet File Open Function 
    bool LoadPcap(int32_t port_num);

    UDPSocket udp_socket_;
    ModelId lidar_id_;
    int port_num_;
    InputType input_type_;
    queue<LidarUdpPacket> packet_queue;
    mutex queue_mutex;
    mutex mtx;
    condition_variable cv;

    int fps = 0;
    int last_fps = 0;
    int lost_packet = 0;
    int last_lost_packet = 0;
    bool is_ready_input_;
    unsigned int fov_data_arr_count_ = 0;
    float top_bottom_offset;
    int data_points_size_;
    float vertical_angle_arr_[56];
    float vert_angle = 10;
    unsigned int frame_rate = 10;
    unsigned int vertical_angle = 10;


    //vector<AutoLG32FovDataBlock> fov_data_set_t;
    int stage_count = 0;
    long long cur_packet_id = 0;
    long long prev_packet_id = 0;
    long long num_of_lost_packet = 0;
    bool is_first_fov_data = true;
    unsigned long long update_count = 0;
    //vector<AutoLG32UdpPacket> frame_data;
    vector<int> lidar_id_vector_;
    
    thread udp_thread;
    thread packets_to_fov_thread;


    // Pcap Variables
    pcap_t *pcap_;
    char filename_[256];
    std::string devip_str_;
    std::string pcap_path_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE] = {0};

    bool stop_udp_thread_ = false;
    bool stop_packets2fov_thread_ = false;

    std::chrono::system_clock::time_point start_vec = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point end_vec;
    
    pthread_t udp_thread_p;
    pthread_t packets_to_fov_thread_p;
    LIDAR_CONFIG lidar_config_;
    int32_t lidar_idx_;

    // calibration
    Calibration calibration_;
    std::vector<SlamOffset> rpy_;

    static void* ThreadEntryPoint(void* arg)
    {
        Parser<LidarUdpPacket>* parser = static_cast<Parser<LidarUdpPacket>*>(arg);
        parser->ReceiveThreadDowork();
        return nullptr;
    }
    static void* ThreadEntryPoint2(void* arg)
    {
        Parser<LidarUdpPacket>* parser = static_cast<Parser<LidarUdpPacket>*>(arg);
        parser->ChangePacketsToFov();
        return nullptr;
    }
    void SetVerticalAngle(ModelId device_id, float angle)
    {
        switch (device_id)
        {
        case ModelId::G32:
        {
            int num_of_channel = 16;
            top_bottom_offset = angle / (2 * num_of_channel);
            float angle_start = -angle / 2 + top_bottom_offset / 2;

            for (int32_t i = 0; i < num_of_channel; i++)
            {
                vertical_angle_arr_[i] = angle_start + (angle / num_of_channel) * i;
                vertical_angle_arr_[i + 16] = angle_start + (angle / num_of_channel) * i + top_bottom_offset;
            }
            
        }
        break;
        case ModelId::S56:
        {            
            int num_of_channel = 56;
            //std::cout << num_of_channel << std::endl;
            for (size_t i = 0; i < num_of_channel; i++)
            {
                vertical_angle_arr_[i] = -15 + ((float)30 / (56 - 1)) * i;
            }
        }
        break;
        default:
            break;
        }
    }
};

template <class LidarUdpPacket>
void Parser<LidarUdpPacket>::StartParserThread(LIDAR_CONFIG &lidar_config, int32_t lidar_idx)
{
    // init configuration
    lidar_config_ = lidar_config;
    input_type_ = lidar_config.input_type;
    lidar_id_ = lidar_config_.model_id;
    port_num_ = lidar_config_.lidar_port[lidar_idx];
    pcap_path_ = lidar_config.pcap_path;
    lidar_idx_ = lidar_idx;
    stop_udp_thread_ = false;
    stop_packets2fov_thread_ = false;

    // Declare thread function for receive packet data through pcap or socket
    //udp_thread = std::thread(&Parser::ReceiveThreadDowork, this);
    // packets_to_fov_thread = std::thread(&Parser::ChangePacketsToFov, this);

    struct sched_param param1;
    int policy= SCHED_FIFO;
    param1.sched_priority = sched_get_priority_max(policy);
    pthread_create(&udp_thread_p, nullptr, &Parser<LidarUdpPacket>::ThreadEntryPoint, this);
    pthread_setschedparam(udp_thread_p, policy, &param1);

    struct sched_param param2;
    int policy2= SCHED_FIFO;
    param2.sched_priority = sched_get_priority_max(policy2);
    pthread_create(&packets_to_fov_thread_p, nullptr, &Parser<LidarUdpPacket>::ThreadEntryPoint2, this);
    pthread_setschedparam(packets_to_fov_thread_p, policy2, &param2);


    // Set calibration config
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::string filePath = "params/slam_offset.yaml";
    std::filesystem::path fullPath = currentPath / filePath;
    
    if(std::filesystem::exists(fullPath) == false)
    {
        std::string tempPath = "autol_driver";
        fullPath = currentPath / tempPath / filePath;
    }
   
    // Read slam_offset.yaml file
    calibration_.ReadSlamOffset(fullPath.string(), lidar_config_.lidar_count);
}

// Stop parser thread
template <class LidarUdpPacket>
void Parser<LidarUdpPacket>::StopParserThread()
{
    stop_packets2fov_thread_ = true;
    stop_udp_thread_ = true;
    
    pthread_join(udp_thread_p, nullptr);
    pthread_join(packets_to_fov_thread_p, nullptr);
}

//Socket Connect Function
template <class LidarUdpPacket>
bool Parser<LidarUdpPacket>::ConnectSocket(int port_num)
{
    int result = udp_socket_.CreateSocket();
    if (result == 0)
    {
        result = udp_socket_.Bind(port_num);
    }

    udp_socket_.SetSocketBuffer(BUF_SIZE);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 3;

    udp_socket_.SetTimeout(tv);

    if (result < 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

template <class LidarUdpPacket>
bool Parser<LidarUdpPacket>::LoadPcap(int32_t port_num)
{
    if ((pcap_ = pcap_open_offline(pcap_path_.c_str(), errbuf_)) == NULL)
    {
        cout << "Error opening AutoL socket dump file \n";
        return false;
    }
    else
    {
        is_ready_input_ = true;
    }

    std::stringstream filter;
    if (devip_str_ != "") // using specific IP?
    {
        filter << "src host " << devip_str_ << " && ";
    }
    filter << "udp dst port " << port_num;
    pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1,
                 PCAP_NETMASK_UNKNOWN);

    return is_ready_input_;
}

// receive data process thread
template <class LidarUdpPacket>
void Parser<LidarUdpPacket>::ReceiveThreadDowork()
{
    sockaddr_in from;
    const int32_t pcapHeaderSz = 42;
    struct pcap_pkthdr *header;
    const u_char *pkt_data;
    char buffer[1500] = {0};
    int32_t retVal = 0;
    int32_t closeFlag = 0;
    //1. Connect the lidar or Open the pcap file
    if (input_type_ == InputType::UDP)
    {
        retVal = ConnectSocket(port_num_);
    }
    else if (input_type_ == InputType::PCAP)
    {
        retVal = LoadPcap(port_num_);
    }

    if (retVal == false)
    {
        return;
    }
    //2. Receive packet data 
    while (stop_udp_thread_ == false)
    {
        //AutoLG32UdpPacket lidar_udp_packet;
        LidarUdpPacket lidar_udp_packet;
        if (input_type_ == InputType::UDP)
        {
            retVal = udp_socket_.RecvFrom(buffer, sizeof(buffer), &from);
        }
        else if (input_type_ == InputType::PCAP)
        {
            retVal = pcap_next_ex(pcap_, &header, &pkt_data);
            if (header->caplen < 0 || retVal < 0)
            {
                pcap_close(pcap_);
                if (lidar_config_.read_once == 0)
                {
                    RCLCPP_INFO(node_->get_logger(), "Replay");
                    LoadPcap(port_num_);
                }
                else
                {
                    closeFlag = 1;
                    RCLCPP_INFO(node_->get_logger(), "End of the File");
                    break;
                }
            }

            retVal = pcap_offline_filter(&pcap_packet_filter_, header, pkt_data);
            if (lidar_config_.read_fast == 0)
            {
                int frame_rate = lidar_config_.frame_rate;
                int packet_per_frame = lidar_config_.packet_per_frame;
                
                usleep((float)1/frame_rate/packet_per_frame * 1000000.0);
            }
        }
        if (retVal <= 0)
        {
            continue;
        }

        //copy the data memory for data type
        if (pkt_data != NULL && input_type_ == InputType::PCAP)
        {
            memcpy(buffer, pkt_data + pcapHeaderSz, header->caplen);
            usleep(100); //for thread sync
        }
        
        lidar_udp_packet.DeSerializeUdpPacket(buffer);
        //3. accumulate packet data to packet_queue 
        //ChangePacketsToFovSyn(lidar_udp_packet, lidar_idx_);
        //queue_mutex.lock();
        unique_lock<mutex> lock(mtx);
        if (packet_queue.size() < 200)
        {                
            packet_queue.push(lidar_udp_packet);                
        }
        //queue_mutex.unlock();
        cv.notify_one();

    }
    if (input_type_ == InputType::UDP)
    {
        udp_socket_.CloseSocket();
    }

    else if (input_type_ == InputType::PCAP && closeFlag == 0)
    {
        pcap_close(pcap_);
    }
    RCLCPP_INFO(node_->get_logger(), "Stop Recv Packet");
}

#endif