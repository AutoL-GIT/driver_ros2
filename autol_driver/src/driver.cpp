#include "define.h"

namespace autol_driver
{
  AutolDriver::AutolDriver(const rclcpp::NodeOptions &node_options)
      : Node("autol_driver_node", node_options)
  {
    for (int i = 0; i < 6; ++i)
    {
      PORT_LIST[i] = 5001 + i;
    }
    
    string pcap_path;
    // default Parameter
    this->declare_parameter("manufacture_id", rclcpp::ParameterValue("autol"));
    this->declare_parameter("model_id", rclcpp::ParameterValue("G32"));
    this->declare_parameter("input_type", input_type_);
    this->declare_parameter("pcap_path", rclcpp::ParameterValue(""));
    this->declare_parameter("packet_per_frame", rclcpp::ParameterValue(180));
    this->declare_parameter("framerate", rclcpp::ParameterValue(25));
    this->declare_parameter("lidar_count", rclcpp::ParameterValue(1));

    this->declare_parameter("lidar_port_1", rclcpp::ParameterValue(5001));
    this->declare_parameter("lidar_port_2", rclcpp::ParameterValue(5002));
    this->declare_parameter("lidar_port_3", rclcpp::ParameterValue(5003));
    this->declare_parameter("lidar_port_4", rclcpp::ParameterValue(5004));
    this->declare_parameter("lidar_port_5", rclcpp::ParameterValue(5005));
    this->declare_parameter("lidar_port_6", rclcpp::ParameterValue(5006));

    // get Parameter to Config
    this->get_parameter("manufacture_id", config_.manufacture_id);
    this->get_parameter("model_id", config_.model);
    this->get_parameter("input_type", input_type_);
    this->get_parameter("pcap_path", pcap_path);
    this->get_parameter("packet_per_frame", PACKET_PER_SECOND);
    this->get_parameter("frame_rate", FRAMERATE);
    this->get_parameter("lidar_count", LIDAR_COUNT);

    this->get_parameter("lidar_port_1", PORT_LIST[0]);
    this->get_parameter("lidar_port_2", PORT_LIST[1]);
    this->get_parameter("lidar_port_3", PORT_LIST[2]);
    this->get_parameter("lidar_port_4", PORT_LIST[3]);
    this->get_parameter("lidar_port_5", PORT_LIST[4]);
    this->get_parameter("lidar_port_6", PORT_LIST[5]);

    if (LIDAR_COUNT < 1)
    {
      LIDAR_COUNT = 1;
    }
    if (LIDAR_COUNT > 6)
    {
      LIDAR_COUNT = 6;
    }

    is_packet_init = false;

    pub_frame_ = this->create_publisher<autol_msgs::msg::AutolFrame>("autol_frame_data", 10);

    if (input_type_ == 1)
    {
      for (int i = 0; i < LIDAR_COUNT; ++i)
      {
        input_data_[i].reset(
            new autol_driver::SocketInput(this, PORT_LIST[i]));
      }
    }
    else if (input_type_ == 2)
    {
      for (int i = 0; i < 1; ++i)
      {
        input_data_[0].reset(new autol_driver::PcapInput(
            this, PORT_LIST[0], FRAMERATE * PACKET_PER_SECOND, pcap_path));
      }
    }
    else if (input_type_ == 3)
    {
      ;
    }
  }

  bool AutolDriver::Dispose() { return true; }

  void AutolDriver::StartRecvPacket()
  {
    if (input_type_ == 1)
    {

      for (int i = 0; i < LIDAR_COUNT; ++i)
      {
        thread_pool[i] = std::thread(&AutolDriver::RecvPacket, this, i);
      }
      for (int i = 0; i < LIDAR_COUNT; ++i)
      {
        thread_pool[i].join();
      }
    }
    else if (input_type_ == 2)
    {
      thread_pool[0] = std::thread(&AutolDriver::RecvPacket, this, 0);
      thread_pool[0].join();
    }
  }

  void AutolDriver::RecvPacket(int lidar_index)
  {
    if (input_type_ == 3)
    {
      return;
    }
    UdpPacket *packet = new UdpPacket();
    autol_msgs::msg::AutolPacket tmp_packet;
    auto lidar_frame = std::make_shared<autol_msgs::msg::AutolFrame>();
    auto lidar1_frame = std::make_shared<autol_msgs::msg::AutolFrame>();
    auto lidar2_frame = std::make_shared<autol_msgs::msg::AutolFrame>();

    bool is_lidar1_active = false;
    bool is_lidar2_active = false;
    unsigned int lidar1_frame_count = 0;
    unsigned int lidar2_frame_count = 0;

    bool is_first_fov_lidar1 = true;
    bool is_first_fov_lidar2 = true;
    int lidar1_stage_count = 0;
    int lidar2_stage_count = 0;

    lidar_frame->packets.reserve(PACKET_PER_SECOND);
    lidar1_frame->packets.reserve(PACKET_PER_SECOND);
    lidar2_frame->packets.reserve(PACKET_PER_SECOND);

    unsigned long pack_id_cnt;
    unsigned long prev_packet_id = 0;

    is_packet_init = false;

    bool is_packet_lost = false;
    while (1)
    {
      if (input_data_[lidar_index]->is_ready_input_ == false)
      {
        break;
      }
      int ret =
          input_data_[lidar_index]->getPacket(&tmp_packet, PACKET_DATA_SIZE);
      if (ret != PACKET_DATA_SIZE)
      {
        RCLCPP_INFO(this->get_logger(), "error :%d", lidar_index);
        continue;
      }

      memcpy(packet, (char *)&(&tmp_packet)->data[0], PACKET_DATA_SIZE);

      if (is_packet_init == false)
      {
        if (packet->header_.data_type_ == 0xA5B3C2AA &&
            packet->header_.top_bottom_side_ == 1)
        {
          PACKET_PER_SECOND = packet->header_.packet_id_ + 1;
          is_packet_init = true;
        }
        continue;
      }
      pack_id_cnt = packet->header_.packet_id_;

      if (pack_id_cnt != prev_packet_id + 1)
      {
        if (pack_id_cnt != 0)
        {
          is_packet_lost = true;
          RCLCPP_INFO(this->get_logger(), "packet lost");
        }
      }
      prev_packet_id = pack_id_cnt;

      if (packet->factory_ == 0x11)
      {
        if (!(lidar1_frame->packets.size() == 0 &&
              packet->header_.top_bottom_side_ == 1))
        {
          is_lidar1_active = true;
          lidar1_frame->packets.push_back(tmp_packet);
        }
      }
      else if (packet->factory_ == 0x12)
      {
        if (!(lidar2_frame->packets.size() == 0 &&
              packet->header_.top_bottom_side_ == 1))
        {
          is_lidar2_active = true;
          lidar2_frame->packets.push_back(tmp_packet);
        }
      }

      if (packet->header_.data_type_ == 0xA5B3C2AA &&
          packet->header_.packet_id_ != 0)
      {
        if (packet->factory_ == 0x11)
        {
          if (!(lidar1_frame->packets.size() == 0 &&
                packet->header_.top_bottom_side_ == 1))
          {
            ++lidar1_stage_count;
          }

          if (lidar1_stage_count >= 2)
          {
            if (lidar1_frame_count == 0)
            {
              lidar_frame->packets.insert(lidar_frame->packets.end(),
                                          lidar1_frame->packets.begin(),
                                          lidar1_frame->packets.end());
            }
            lidar1_frame.reset(new autol_msgs::msg::AutolFrame);
            lidar1_frame->packets.reserve(PACKET_PER_SECOND);

            lidar1_stage_count = 0;
            ++lidar1_frame_count;
          }
        }
        else if (packet->factory_ == 0x12)
        {
          if (!(lidar2_frame->packets.size() == 0 &&
                packet->header_.top_bottom_side_ == 1))
          {
            ++lidar2_stage_count;
          }

          if (lidar2_stage_count >= 2)
          {
            if (lidar2_frame_count == 0)
            {
              lidar_frame->packets.insert(lidar_frame->packets.end(),
                                          lidar2_frame->packets.begin(),
                                          lidar2_frame->packets.end());
            }
            lidar2_frame.reset(new autol_msgs::msg::AutolFrame);
            lidar2_frame->packets.reserve(PACKET_PER_SECOND);

            lidar2_stage_count = 0;
            ++lidar2_frame_count;
          }
        }

        if (lidar1_frame_count > 50 || lidar2_frame_count > 50)
        {
          lidar1_frame_count = 0;
          lidar2_frame_count = 0;

          int active_lidar_count = 2;
          if (lidar1_frame_count == 0)
          {
            is_lidar1_active = false;
            --active_lidar_count;
          }
          if (lidar2_frame_count == 0)
          {
            is_lidar2_active = false;
            --active_lidar_count;
          }
          lidar_frame.reset(new autol_msgs::msg::AutolFrame);
          lidar_frame->packets.reserve(active_lidar_count * PACKET_PER_SECOND);
        }

        if (((is_lidar1_active == true && lidar1_frame_count > 0) ||
             is_lidar1_active == false) &&
            ((is_lidar2_active == true && lidar2_frame_count > 0) ||
             is_lidar2_active == false) &&
            (is_lidar1_active == true || is_lidar2_active == true))
        {
          int active_lidar_count = 0;

          if (is_lidar1_active == true && is_lidar2_active == true)
          {
            input_data_[lidar_index]->changePacketRate(FRAMERATE *
                                                       PACKET_PER_SECOND * 2);
            active_lidar_count = 2;
          }
          else
          {
            input_data_[lidar_index]->changePacketRate(FRAMERATE *
                                                       PACKET_PER_SECOND * 1);
            if (is_lidar1_active == true)
              ++active_lidar_count;
            if (is_lidar2_active == true)
              ++active_lidar_count;
          }

          if (lidar_frame->packets.size() ==
              active_lidar_count * PACKET_PER_SECOND)
          {

            lidar_frame->lidar_index = lidar_index;
            pub_frame_->publish(*lidar_frame);
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "skip publish : %ld", lidar_frame->packets.size());
          }
          lidar_frame = std::make_shared<autol_msgs::msg::AutolFrame>();
          lidar_frame->packets.reserve(active_lidar_count * PACKET_PER_SECOND);

          lidar1_frame_count = 0;
          lidar2_frame_count = 0;
        }

        is_packet_lost = false;
      }
    }

    delete packet;
    packet = NULL;
  }

} // namespace autol_driver
