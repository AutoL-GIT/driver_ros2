#ifndef AUTOL_ROS_Driver_HPP_
#define AUTOL_ROS_Driver_HPP_

#include "define.hpp"
#include "input/input_manager.hpp"
#include "input/input_socket.hpp"
#include "input/input_pcap.hpp"

#include <sstream>
#include <chrono>

class AutolDriver : public rclcpp::Node
{
public:
  AutolDriver(const rclcpp::NodeOptions &options);
  ~AutolDriver() {}
  // Initialize some necessary configuration parameters, create ROS nodes, and register callback functions
  virtual void Init();
  // Start working
  virtual void Start();
  // Stop working
  virtual void Stop() {}
  // Used to publish the original packet pub_frame_
  void SendPacketG32V1(const G32FrameData_t &autol_frame, int32_t lidar_idx);
  void SendPacketG32V2(const G32V2FrameData_t &autol_frame, int32_t lidar_idx);
  void SendPacketS56(const S56FrameData_t &autol_frame, int32_t lidar_idx);
  // Used to publish point clouds
  void SendPcdData(const PointData &fov_data_set, int32_t lidar_idx);

  void PcdPublishThreadDowork(const PointData fov_data_set, int32_t lidar_idx);

protected:
  //Packet Frame Publisher
  rclcpp::Publisher<autol_driver::msg::AutolG32Frame>::SharedPtr pub_g32_frame_[MAX_NUM_LIDAR];
  rclcpp::Publisher<autol_driver::msg::AutolG32V2Frame>::SharedPtr pub_g32_v2_frame_[MAX_NUM_LIDAR];
  rclcpp::Publisher<autol_driver::msg::AutolS56Frame>::SharedPtr pub_s56_frame_[MAX_NUM_LIDAR];
  //Point Cloud Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_[MAX_NUM_LIDAR];

private:
  rclcpp::Node &node_;
  // lidar configure
  LIDAR_CONFIG lidar_config_;
  // Convert packets into ROS messages
  std::vector<InputManager::Ptr> input_ptr_;
  // Merge each lidar point cloud
  std::map<int32_t, PointData> merge_pcd_data_;
  //point cloud msg
  sensor_msgs::msg::PointCloud2 ros_msg_arr_[MAX_NUM_LIDAR];
  mutex publish_mutex;

};

AutolDriver::AutolDriver(const rclcpp::NodeOptions &node_options)
    : Node("autol_driver_node", node_options), node_(*this)
{
}

void AutolDriver::Init()
{
  int32_t temp_port = 0;
  int32_t temp_type = 0;
  string manufacture_id;
  string temp_model_id;
  // default Parameter
  node_.declare_parameter("manufacture_id", rclcpp::ParameterValue("autol"));
  node_.declare_parameter("model_id", rclcpp::ParameterValue("G32"));
  node_.declare_parameter("data_format_version", rclcpp::ParameterValue(4));
  node_.declare_parameter("input_type", rclcpp::ParameterValue(1));
  

  node_.declare_parameter("lidar_count", rclcpp::ParameterValue(1));
  node_.declare_parameter("lidar_port_1", rclcpp::ParameterValue(5001));
  node_.declare_parameter("lidar_port_2", rclcpp::ParameterValue(5002));
  node_.declare_parameter("lidar_port_3", rclcpp::ParameterValue(5003));
  node_.declare_parameter("lidar_port_4", rclcpp::ParameterValue(5004));
  node_.declare_parameter("lidar_port_5", rclcpp::ParameterValue(5005));
  node_.declare_parameter("lidar_port_6", rclcpp::ParameterValue(5006));

  node_.declare_parameter("pcap_path", rclcpp::ParameterValue(""));
  node_.declare_parameter("frame_rate", rclcpp::ParameterValue(10));
  node_.declare_parameter("packet_per_frame", rclcpp::ParameterValue(180));
  node_.declare_parameter("read_once", rclcpp::ParameterValue(0));
  node_.declare_parameter("read_fast", rclcpp::ParameterValue(0));

  node_.declare_parameter("calibration", rclcpp::ParameterValue(true));

  // get Parameter to Config
  node_.get_parameter("manufacture_id", manufacture_id);
  node_.get_parameter("model_id", temp_model_id);
  node_.get_parameter("data_format_version", lidar_config_.data_format_version);
  node_.get_parameter("input_type", temp_type);
  
  node_.get_parameter("lidar_count", lidar_config_.lidar_count);
  node_.get_parameter("lidar_port_1", temp_port);
  lidar_config_.lidar_port.push_back(temp_port);
  node_.get_parameter("lidar_port_2", temp_port);
  lidar_config_.lidar_port.push_back(temp_port);
  node_.get_parameter("lidar_port_3", temp_port);
  lidar_config_.lidar_port.push_back(temp_port);
  node_.get_parameter("lidar_port_4", temp_port);
  lidar_config_.lidar_port.push_back(temp_port);
  node_.get_parameter("lidar_port_5", temp_port);
  lidar_config_.lidar_port.push_back(temp_port);
  node_.get_parameter("lidar_port_6", temp_port);
  lidar_config_.lidar_port.push_back(temp_port);

  // get Pcap Parameter to Config
  node_.get_parameter("pcap_path", lidar_config_.pcap_path);
  node_.get_parameter("frame_rate", lidar_config_.frame_rate);
  node_.get_parameter("packet_per_frame", lidar_config_.packet_per_frame);
  node_.get_parameter("read_once", lidar_config_.read_once);
  node_.get_parameter("read_fast", lidar_config_.read_fast);
  node_.get_parameter("calibration", lidar_config_.calibration);


  //Set Lidar Configuration
  if (manufacture_id == "autol")
  {
    lidar_config_.manufacture_id = ManufatureId::autol;
  }
  else
  {
    RCLCPP_INFO(node_.get_logger(), "Invalid Lidar Manufacture id");
  }

  if (temp_model_id == "G32")
  {
    lidar_config_.model_id = ModelId::G32;
    if(lidar_config_.packet_per_frame  == 0)
    {
      if(lidar_config_.data_format_version == 1)
        lidar_config_.packet_per_frame = 172;
      else if (lidar_config_.data_format_version == 2)
        lidar_config_.packet_per_frame = 206;
    }
  }
  else if (temp_model_id == "S56")
  {
    lidar_config_.model_id = ModelId::S56;
    if(lidar_config_.packet_per_frame  == 0)
    {
      lidar_config_.packet_per_frame = 192;
    }
  }
  else
  {
    RCLCPP_INFO(node_.get_logger(), "Invalid Lidar Model id");
  }
  if (temp_type == 1)
  {
    lidar_config_.input_type = InputType::UDP;
  }
  else if (temp_type == 2)
  {
    lidar_config_.input_type = InputType::PCAP;
  }
  else
  {
    RCLCPP_INFO(node_.get_logger(), "Invalid Input Type");
  }

  // Lidar Count Exception
  if (lidar_config_.lidar_count < 1)
  {
    lidar_config_.lidar_count = 1;
  }
  else if (lidar_config_.lidar_count > 6)
  {
    lidar_config_.lidar_count = 6;
  }

  for (int32_t lidar_idx = 0; lidar_idx < lidar_config_.lidar_count; lidar_idx++)
  {
    std::shared_ptr<InputManager> input_ptr;
    switch (lidar_config_.input_type)
    {
    case InputType::UDP:
      input_ptr = std::make_shared<InputSocket>(lidar_config_, lidar_idx);
      break;
    case InputType::PCAP:
      input_ptr = std::make_shared<InputPcap>(lidar_config_, lidar_idx);
      break;
    default:
      RCLCPP_INFO(node_.get_logger(), "Invalid Input Type !");
      break;
    }
    // Send Packet Data Callback Function
    input_ptr->RegRecvCallback(std::bind(&AutolDriver::SendPacketG32V1, this, std::placeholders::_1, std::placeholders::_2));
    input_ptr->RegRecvCallback(std::bind(&AutolDriver::SendPacketG32V2, this, std::placeholders::_1, std::placeholders::_2));
    input_ptr->RegRecvCallback(std::bind(&AutolDriver::SendPacketS56, this, std::placeholders::_1, std::placeholders::_2));

    // Send Pcd Data Callback Function
    input_ptr->RegRecvPcdCallback(std::bind(&AutolDriver::SendPcdData, this, std::placeholders::_1, std::placeholders::_2));
    input_ptr_.emplace_back(input_ptr);
  }

  for(int32_t iIdxI = 0; iIdxI < lidar_config_.lidar_count; iIdxI++)
  {
    std::ostringstream oss_1;
    std::ostringstream oss_2;
    oss_1 << "autol_frame_data_" << iIdxI + 1;
    oss_2 << "autol_pointcloud_" << iIdxI + 1;
    std::string frame_data = oss_1.str();
    std::string poin_cloud = oss_2.str();;
    switch(lidar_config_.model_id )
    {
      case ModelId::G32:
        if(lidar_config_.data_format_version == 1)
          pub_g32_frame_[iIdxI] = node_.create_publisher<autol_driver::msg::AutolG32Frame>(frame_data, 10);
        else if(lidar_config_.data_format_version == 2)
          pub_g32_v2_frame_[iIdxI] = node_.create_publisher<autol_driver::msg::AutolG32V2Frame>(frame_data, 10);
      break;
      case ModelId::S56:
        pub_s56_frame_[iIdxI] = node_.create_publisher<autol_driver::msg::AutolS56Frame>(frame_data, 10);
      break;
    }
    
    pub_pcd_[iIdxI] = node_.create_publisher<sensor_msgs::msg::PointCloud2>(poin_cloud, 10);
  }

  return;
}

void AutolDriver::Start()
{
  for (auto &iter : input_ptr_)
  {
    if (iter != nullptr)
    {
      iter->SetRosNode(&node_); //Get node address
      iter->StartRecvData(); 
    }
  }
}

inline void AutolDriver::SendPacketG32V1(const G32FrameData_t &fov_data_set, int32_t lidar_idx)
{
  autol_driver::msg::AutolG32Frame lidar_frame;
  for (int32_t iIdxI = 0; iIdxI < (int32_t)fov_data_set.size(); iIdxI++)
  {
    autol_driver::msg::AutolG32Packet lidar_packet;
    memcpy(&lidar_packet.data[0], &fov_data_set[iIdxI], sizeof(AutoLG32UdpPacket));
    lidar_frame.packets.emplace_back(lidar_packet);
  }
  lidar_frame.lidar_index = lidar_idx;
  pub_g32_frame_[lidar_idx]->publish(lidar_frame);
}

inline void AutolDriver::SendPacketG32V2(const G32V2FrameData_t &fov_data_set, int32_t lidar_idx)
{
  autol_driver::msg::AutolG32V2Frame lidar_frame;
  for (int32_t iIdxI = 0; iIdxI < (int32_t)fov_data_set.size(); iIdxI++)
  {
    autol_driver::msg::AutolG32V2Packet lidar_packet;
    memcpy(&lidar_packet.data[0], &fov_data_set[iIdxI], sizeof(AutoLG32UdpPacket));
    lidar_frame.packets.emplace_back(lidar_packet);
  }
  lidar_frame.lidar_index = lidar_idx;
  pub_g32_v2_frame_[lidar_idx]->publish(lidar_frame);
}

inline void AutolDriver::SendPacketS56(const S56FrameData_t &fov_data_set, int32_t lidar_idx)
{
  autol_driver::msg::AutolS56Frame lidar_frame;
  for (int32_t iIdxI = 0; iIdxI < (int32_t)fov_data_set.size(); iIdxI++)
  {
    autol_driver::msg::AutolS56Packet lidar_packet;
    memcpy(&lidar_packet.data[0], &fov_data_set[iIdxI], sizeof(AutoLS56UdpPacket));
    lidar_frame.packets.emplace_back(lidar_packet);
  }
  lidar_frame.lidar_index = lidar_idx;
  pub_s56_frame_[lidar_idx]->publish(lidar_frame);
}

inline void AutolDriver::SendPcdData(const PointData &point_cloud, int32_t lidar_idx)
{
  if(true)
  {
    std::thread pcd_publish_thread(&AutolDriver::PcdPublishThreadDowork, this, point_cloud, std::ref(lidar_idx));
    pcd_publish_thread.detach();
    return;
  }
  else
  {
    PcdPublishThreadDowork(point_cloud, lidar_idx);
  }
}

void AutolDriver::PcdPublishThreadDowork(const PointData point_cloud, int32_t lidar_idx)
{
  int32_t offset = 0;
  int32_t fields = 6;
  int32_t point_num_arr[MAX_NUM_LIDAR] = { 0 };

  // step1. lidar update check
  sensor_msgs::msg::PointCloud2 ros_msg_;
  
  // step2. send lidar pcd data
  ros_msg_.fields.clear();
  ros_msg_.fields.reserve(fields);
  ros_msg_.width = point_cloud.size() * 1;
  ros_msg_.height = 1;
  offset = addPointField(ros_msg_, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg_, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg_, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg_, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg_, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg_, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
  ros_msg_.point_step = offset;    
  ros_msg_.row_step = ros_msg_.width * ros_msg_.point_step;
  ros_msg_.is_dense = false;
  ros_msg_.data.resize(point_cloud.size() * 1 * ros_msg_.point_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg_, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg_, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg_, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg_, "timestamp");

  for (int32_t iIdxJ = 0; iIdxJ < (int32_t)point_cloud.size(); iIdxJ++)
  {
    DataPoint point = point_cloud[iIdxJ];
    *iter_x_ = point.x;
    *iter_y_ = point.y;
    *iter_z_ = point.z;
    *iter_intensity_ = point.intensity;
    *iter_ring_ = point.ring;
    *iter_timestamp_ = point.timestamp;
    ++iter_x_;
    ++iter_y_;
    ++iter_z_;
    ++iter_intensity_;
    ++iter_ring_;
    ++iter_timestamp_;
  }

  ros_msg_.header.frame_id = "autol_lidar";

  if(true)
  {
    pub_pcd_[lidar_idx].reset();
    std::ostringstream oss_2;
    oss_2 << "autol_pointcloud_" << lidar_idx + 1;
    std::string poin_cloud = oss_2.str();;
    pub_pcd_[lidar_idx] = node_.create_publisher<sensor_msgs::msg::PointCloud2>(poin_cloud, 10);

    pub_pcd_[lidar_idx]->publish(ros_msg_);
  }
  else
  {
    ros_msg_.header.stamp = node_.now();
    pub_pcd_[lidar_idx]->publish(std::move(ros_msg_));
  }


  if(false)
  {
    static int call_count= 0;
    static auto last_time = std::chrono::steady_clock::now();
    ++call_count;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_time;

    if(elapsed.count() >=1.0)
    {
      cout << call_count<<endl;
      call_count = 0;
      last_time = now;
    }
  }
}

#endif
