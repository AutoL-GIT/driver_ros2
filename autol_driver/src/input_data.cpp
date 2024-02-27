#include "define.h"
#include "driver.h"
#include "input_data.h"
namespace autol_driver
{
  InputData::InputData(rclcpp::Node *node, uint16_t port, double packet_rate)
      : node_(node), port_(port), packet_rate_(rclcpp::Rate(packet_rate)),
        is_ready_input_(false) {}
  SocketInput::SocketInput(rclcpp::Node *node, uint16_t port)
      : node_(node), InputData(node, port)
  {
    if (ConnectUdpCom() == true){
      is_ready_input_ = true;
    }
  }

  bool SocketInput::ConnectUdpCom()
  {
    int ret = 0;
    if (ret == 0)
    {
      ret = udp_socket.CreateSocket();
    }
    if (ret == 0)
    {
      ret = udp_socket.Bind(port_);
    }
    if (ret < 0)
    {
      RCLCPP_INFO(node_->get_logger(), "Bind error %d Port: %d", ret, port_);
    }

    udp_socket.SetSocketBuffer(BUF_SIZE);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 3;

    udp_socket.SetTimeout(tv);

    if (ret < 0)
    {
      RCLCPP_INFO(node_->get_logger(), "Failed UDP Connection");
      return false;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Success UDP Connection Port: %d", port_);
      return true;
    }
  }

  SocketInput::~SocketInput(void)
  {
    RCLCPP_INFO(node_->get_logger(), "close SocketInput()");
    udp_socket.CloseSocket();
  }

  int SocketInput::getPacket(autol_msgs::msg::AutolPacket *packet, int len)
  {
    return udp_socket.RecvFrom(packet, len);
  }

  PcapInput::PcapInput(rclcpp::Node *node, uint16_t port, double packet_rate,
                       std::string filename, bool read_once, bool read_fast,
                       double repeat_delay)
      : node_(node), InputData(node, port, packet_rate), filename_(filename)
  {
    pcap_ = NULL;
    empty_ = true;

    // get parameters using private node handle
    // Set Parameter
    node->declare_parameter("read_once", rclcpp::ParameterValue(false));
    node->declare_parameter("read_fast", rclcpp::ParameterValue(false));
    node->declare_parameter("repeat_delay", rclcpp::ParameterValue(0.0));

    node->get_parameter("read_once", read_once_);
    node->get_parameter("read_fast", read_fast_);
    node->get_parameter("repeat_delay", repeat_delay_);

    if (read_once_)
    {
      RCLCPP_INFO(node->get_logger(), "Read input file only once.");
    }
    if (read_fast_)
    {
      RCLCPP_INFO(node_->get_logger(), "Read input file as quickly as possible.");
    }
    if (repeat_delay_ > 0.0)
    {
      RCLCPP_INFO(node->get_logger(),
                  "Delay %.3f seconds before repeating input file.",
                  repeat_delay_);
    }
    // Open the PCAP dump file
    RCLCPP_INFO(node_->get_logger(), "Opening PCAP file \"%s\"",
                filename_.c_str());
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error opening AutoL socket dump file");
      return;
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
    filter << "udp dst port " << port;
    pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1,
                 PCAP_NETMASK_UNKNOWN);
  }

  /** destructor */
  PcapInput::~PcapInput(void) { pcap_close(pcap_); }

  void PcapInput::changePacketRate(double packet_rate)
  {
    rclcpp::Rate packet_rate_(packet_rate);
  }

  int PcapInput::getPacket(autol_msgs::msg::AutolPacket *pkt, int len)
  {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;

    int packet_cnt = 0;
    while (true)
    {
      int res;
      if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
      {
        // Skip packets not for the correct port and from the
        // selected IP address.
        int ret = pcap_offline_filter(&pcap_packet_filter_, header, pkt_data);

        if (0 == ret)
          continue;

        // Keep the reader from blowing through the file.
        if (read_fast_ == false)
          packet_rate_.sleep();

        memcpy(&pkt->data[0], pkt_data + 42, PACKET_DATA_SIZE);

        empty_ = false;
        return len; // success
      }

      if (empty_) // no data in file?
      {
        RCLCPP_WARN(node_->get_logger(), "Error %d reading packet: %s", res,
                    pcap_geterr(pcap_));
        return -1;
      }

      if (read_once_)
      {
        RCLCPP_INFO(node_->get_logger(), "end of file reached -- done reading.");
        return -1;
      }

      if (repeat_delay_ > 0.0)
      {
        RCLCPP_INFO(node_->get_logger(), "end of file reached -- delaying %.3f seconds.", repeat_delay_);
        usleep(rint(repeat_delay_ * 1000000.0));
      }
      RCLCPP_DEBUG(node_->get_logger(), "replaying AutoL dump file");
      pcap_close(pcap_);
      pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
      empty_ = true;
    }

    return 0;
  }
} // namespace autol_driver
