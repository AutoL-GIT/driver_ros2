#ifndef AUTOL_ROS_DRIVER_H_
#define AUTOL_ROS_DRIVER_H_

#include "define.h"
#include "input_data.h"

namespace autol_driver {
#define BUF_SIZE pow(2, 20) * 1024
class AutolDriver : public rclcpp::Node
{
  public:
    AutolDriver(const rclcpp::NodeOptions &options);
    ~AutolDriver(){};

    bool ConnectUdpCom();

    void StartRecvPacket();
    void RecvPacket(int lidar_index);
    bool Dispose();

  private:
    struct {
      std::string manufacture_id;
      std::string model;
    } config_;

    boost::shared_ptr<InputData> input_data_[6];

    UDPSocket udp_socket;
    rclcpp::Publisher<autol_msgs::msg::AutolFrame>::SharedPtr pub_frame_; 
    bool stop_udp_thread_ = false;
    int input_type_;
    int PACKET_PER_SECOND;
    int FRAMERATE;
    int LIDAR_COUNT;
    int PORT_LIST[6];
    std::thread thread_pool[6];

    bool is_packet_init;
  };
} // namespace autol_driver
#endif
