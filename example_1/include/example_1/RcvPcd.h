
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace example
{
    class RcvPcd : public rclcpp::Node
    {
    public:
        RcvPcd(const rclcpp::NodeOptions &options);
        ~RcvPcd() {}

        void DoSomthing();
        void pointcloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr &pcd);

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPointData_;
    };
}
