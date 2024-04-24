
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
class PcdSaver : public rclcpp::Node
{
public:
    PcdSaver(const rclcpp::NodeOptions &options);
    ~PcdSaver() {}
    void SavePcdCallBack(const sensor_msgs::msg::PointCloud2::ConstPtr &pcd);

private:
    int32_t cnt_;
    std::string save_path_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointData_;
    rclcpp::Node &node_;
};

PcdSaver::PcdSaver(const rclcpp::NodeOptions &options)
    : rclcpp::Node("pcd_saver_node", options), node_(*this)
{
    // get save path
    this->declare_parameter("save_path", rclcpp::ParameterValue(""));
    this->get_parameter("save_path", save_path_);

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    subPointData_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("autol_pointcloud_1",
                                                                             qos_profile,
                                                                             std::bind(&PcdSaver::SavePcdCallBack, this,
                                                                                       std::placeholders::_1));
    rclcpp::spin(this->get_node_base_interface());
}

//Subscribe PointCloud2 Msg and Save the PCD File
void PcdSaver::SavePcdCallBack(const sensor_msgs::msg::PointCloud2::ConstPtr &pcd)
{
    ++cnt_;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //PointCloud2 Msg -> PCD Format
    pcl::fromROSMsg(*pcd, cloud);
    std::stringstream ss;
    ss << save_path_ << cnt_ << ".pcd";

    std::string filename = ss.str();

    RCLCPP_INFO(node_.get_logger(), "pcd data : %s", filename.c_str());
    //Save PCD Format
    pcl::io::savePCDFileASCII(filename, cloud);
}
