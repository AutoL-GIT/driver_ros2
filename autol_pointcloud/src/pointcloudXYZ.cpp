#include "pointcloudXYZ.h"

namespace autol_data
{
    PointcloudXYZ::PointcloudXYZ(rclcpp::Node *node, const double max_range, const double min_range,
                                 const std::string &target_frame, const std::string &fixed_frame,
                                 const unsigned int scans_per_block, std::shared_ptr<tf2_ros::Buffer> tf_ptr)
        : node_(node), ConvertDataBase(node_, max_range, min_range, target_frame, fixed_frame,
                                       0, 1, true, scans_per_block, tf_ptr, 6,
                                       "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "ring", 1, sensor_msgs::msg::PointField::UINT16,
                                       "time", 1, sensor_msgs::msg::PointField::FLOAT32),
          iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(cloud, "z"),
          iter_ring(cloud, "ring"), iter_intensity(cloud, "intensity"), iter_time(cloud, "time")
    {
        RCLCPP_INFO(node->get_logger(), "PointCloudXYZ()");
    };

    void PointcloudXYZ::Setup(const autol_msgs::msg::AutolFrame::ConstPtr &frame_msg)
    {
        ConvertDataBase::Setup(frame_msg);
        iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
        iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
        iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
        iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
        iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring");
        iter_time = sensor_msgs::PointCloud2Iterator<float>(cloud, "time");
    }

    void PointcloudXYZ::insertPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth,
                                    const float distance, const float intensity, const float time)
    {
        *iter_x = x;
        *iter_y = y;
        *iter_z = z;
        *iter_ring = ring;
        *iter_intensity = intensity;
        *iter_time = time;

        ++cloud.width;
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_ring;
        ++iter_intensity;
        ++iter_time;
    }
}
