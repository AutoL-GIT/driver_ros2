#ifndef AUTOL_ROS_Driver_HPP_
#define AUTOL_ROS_Driver_HPP_

#include "define.hpp"
#include "input/input_manager.hpp"
#include "input/input_socket.hpp"
#include "input/input_pcap.hpp"
#include "packet_structure/g32_packet_structure.hpp"

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
  // Used to publish the original packet 
  void SendPacket(const G32FrameData_t &autol_frame, int32_t lidar_idx);
  // Used to publish point clouds
  void SendPcdData(const G32PointData &fov_data_set, int32_t lidar_idx);

protected:
  rclcpp::Publisher<autol_driver::msg::AutolFrame>::SharedPtr pub_frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;

private:
  rclcpp::Node &node_;
  // lidar configure
  LIDAR_CONFIG lidar_config_;
  // Convert packets into ROS messages
  std::vector<InputManager::Ptr> input_ptr_;
  // Merge each lidar point cloud
  std::map<int32_t, G32PointData> merge_pcd_data_;
  //point cloud msg
  sensor_msgs::msg::PointCloud2 ros_msg_;

};

AutolDriver::AutolDriver(const rclcpp::NodeOptions &node_options)
    : Node("autol_driver_node", node_options), node_(*this)
{
  ;
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
  node_.declare_parameter("input_type", rclcpp::ParameterValue(1));
  node_.declare_parameter("framerate", rclcpp::ParameterValue(25));

  node_.declare_parameter("lidar_count", rclcpp::ParameterValue(1));
  node_.declare_parameter("lidar_port_1", rclcpp::ParameterValue(5001));
  node_.declare_parameter("lidar_port_2", rclcpp::ParameterValue(5002));
  node_.declare_parameter("lidar_port_3", rclcpp::ParameterValue(5003));
  node_.declare_parameter("lidar_port_4", rclcpp::ParameterValue(5004));
  node_.declare_parameter("lidar_port_5", rclcpp::ParameterValue(5005));
  node_.declare_parameter("lidar_port_6", rclcpp::ParameterValue(5006));

  node_.declare_parameter("pcap_path", rclcpp::ParameterValue(""));
  node_.declare_parameter("packet_per_frame", rclcpp::ParameterValue(180));
  node_.declare_parameter("read_once", rclcpp::ParameterValue(0));
  node_.declare_parameter("read_fast", rclcpp::ParameterValue(0));
  node_.declare_parameter("repeat_delay", rclcpp::ParameterValue(0.0));

  node_.declare_parameter("calibration", rclcpp::ParameterValue(true));

  // get Parameter to Config
  node_.get_parameter("manufacture_id", manufacture_id);
  node_.get_parameter("model_id", temp_model_id);
  node_.get_parameter("input_type", temp_type);
  node_.get_parameter("frame_rate", lidar_config_.frame_rate);

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
  node_.get_parameter("packet_per_frame", lidar_config_.packet_per_frame);
  node_.get_parameter("read_once", lidar_config_.read_once);
  node_.get_parameter("read_fast", lidar_config_.read_fast);
  node_.get_parameter("repeat_delay", lidar_config_.repeat_delay);
  node_.get_parameter("calibration", lidar_config_.calibration);

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
    input_ptr->RegRecvCallback(std::bind(&AutolDriver::SendPacket, this, std::placeholders::_1, std::placeholders::_2));
    // Send Pcd Data Callback Function
    input_ptr->RegRecvPcdCallback(std::bind(&AutolDriver::SendPcdData, this, std::placeholders::_1, std::placeholders::_2));
    input_ptr_.emplace_back(input_ptr);
  }
  pub_frame_ = node_.create_publisher<autol_driver::msg::AutolFrame>("autol_frame_data", 10);
  pub_pcd_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("autol_pointcloud", 10);

  return;
}

void AutolDriver::Start()
{
  for (auto &iter : input_ptr_)
  {
    if (iter != nullptr)
    {
      iter->SetRosNode(&node_);
      iter->StartRecvData();
    }
  }
}

inline void AutolDriver::SendPacket(const G32FrameData_t &fov_data_set, int32_t lidar_idx)
{
  autol_driver::msg::AutolFrame lidar_frame;
  for (int32_t iIdxI = 0; iIdxI < (int32_t)fov_data_set.size(); iIdxI++)
  {
    autol_driver::msg::AutolPacket lidar_packet;
    memcpy(&lidar_packet.data[0], &fov_data_set[iIdxI], sizeof(AutoLG32UdpPacket));
    lidar_frame.packets.emplace_back(lidar_packet);
  }
  pub_frame_->publish(lidar_frame);
}

inline void AutolDriver::SendPcdData(const G32PointData &point_cloud, int32_t lidar_idx)
{
  int32_t offset = 0;
  int32_t fields = 6;
  int32_t point_num = 0;

  // step1. lidar update check
  merge_pcd_data_[lidar_idx] = point_cloud;
  for (int32_t iIdxI = 0; iIdxI < lidar_config_.lidar_count; iIdxI++)
  {
    if (merge_pcd_data_[iIdxI].size() == 0)
    {
      return;
    }
    point_num += merge_pcd_data_[iIdxI].size();
  }

  // step2. send lidar pcd data
  ros_msg_.data.clear();
  ros_msg_.fields.clear();
  ros_msg_.fields.reserve(fields);
  ros_msg_.width = point_num;
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
  ros_msg_.data.resize(point_num * ros_msg_.point_step);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg_, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg_, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg_, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg_, "timestamp");

  for (int32_t iIdxI = 0; iIdxI < lidar_config_.lidar_count; iIdxI++)
  {
    for (int32_t iIdxJ = 0; iIdxJ < (int32_t)merge_pcd_data_[iIdxI].size(); iIdxJ++)
    {
      DataPoint point = merge_pcd_data_[iIdxI][iIdxJ];
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
    merge_pcd_data_[iIdxI].clear();
  }

  ros_msg_.header.frame_id = "autol_lidar";
  RCLCPP_INFO(node_.get_logger(), "publish pointcloud data: %d", ros_msg_.width);
  pub_pcd_->publish(ros_msg_);
}
#endif
