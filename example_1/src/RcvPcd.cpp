#include "example_1//RcvPcd.h"

namespace example
{
    RcvPcd::RcvPcd(const rclcpp::NodeOptions &options)
        : rclcpp::Node("example_node", options)
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        subPointData_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("autol_pointcloud_1",
                                                                                 qos_profile,
                                                                                 std::bind(&RcvPcd::pointcloud_cb, this,
                                                                                           std::placeholders::_1));
        rclcpp::spin(this->get_node_base_interface());
    }

    void RcvPcd::DoSomthing()
    {
        // Put your code here
    }

    void RcvPcd::pointcloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr &pcd)
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pcd, "x"),
                                                     iter_y(*pcd, "y"),
                                                     iter_z(*pcd, "z"),
                                                     iter_intensity(*pcd, "intensity");
        int cnt = 0;
        while (iter_x != iter_x.end())
        {
            ++cnt;
            if (cnt == 10000)
            {
                RCLCPP_INFO(this->get_logger(), "%f %f %f %f", *iter_x, *iter_y, *iter_z, *iter_intensity);
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
        }
    }
}
