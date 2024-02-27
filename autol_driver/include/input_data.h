#include "define.h"

#include "udp_socket.h"
#ifndef AUTOL_ROS_INPUT_DATA_H_
#define AUTOL_ROS_INPUT_DATA_H_
#define BUF_SIZE pow(2, 20) * 1024
#define PACKET_DATA_SIZE 1330
#define UDP_PORT 5001
namespace autol_driver
{

  class InputData
  {
  public:
    InputData(rclcpp::Node *node, uint16_t port, double packet_rate = 1.0);
    virtual ~InputData() {}
    virtual int getPacket(autol_msgs::msg::AutolPacket *packet, int len) = 0;
    virtual void changePacketRate(double packet_rate) = 0;
    bool is_ready_input_;

  protected:
    rclcpp::Node *node_;
    rclcpp::Rate packet_rate_;
    uint16_t port_;
    std::string devip_str_;
    bool gps_time_;
  };

  class SocketInput : public InputData
  {
  public:
    SocketInput(rclcpp::Node *node, uint16_t port = UDP_PORT);
    virtual ~SocketInput();
    virtual int getPacket(autol_msgs::msg::AutolPacket *packet, int len);
    bool ConnectUdpCom();
    void setDeviceIP(const std::string &ip);
    virtual void changePacketRate(double packet_rate) {}

  private:
    UDPSocket udp_socket;
    rclcpp::Node *node_;
  };

  class PcapInput : public InputData
  {
  public:
    PcapInput(rclcpp::Node *node, uint16_t port = UDP_PORT,
              double packet_rate = 0.0, std::string filename = "",
              bool read_once = false, bool read_fast = false,
              double repeat_delay = 0.0);
    virtual ~PcapInput();

    virtual int getPacket(autol_msgs::msg::AutolPacket *packet, int len);
    virtual void changePacketRate(double packet_rate);

  private:
    std::string filename_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    bool empty_;
    bool read_once_;
    bool read_fast_;
    double repeat_delay_;
    rclcpp::Node *node_;
  };
} // namespace autol_driver
#endif
