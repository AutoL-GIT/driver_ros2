
#ifndef AUTOL_ROS_CONVERT_H_
#define AUTOL_ROS_CONVERT_H_

#include "define.h"
#include "rawdata.h"
#include "convertdatabase.h"
#include "pointcloudXYZ.h"
#include "calibration.h"

namespace autol_pointcloud
{
    class Convert : public rclcpp::Node
    {
    public:
        Convert(const rclcpp::NodeOptions &options);
        ~Convert() {}

    private:
        void frameData_cb(const autol_msgs::msg::AutolFrame::ConstPtr &frameMsg);
        std::shared_ptr<autol_data::RawData> raw_data_;
        std::shared_ptr<autol_data::ConvertDataBase> convert_data_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_autol_pointcloud_;
        rclcpp::Subscription<autol_msgs::msg::AutolFrame>::SharedPtr sub_frame_;
        bool is_slam_active_ = false;
        int frame_cnt = 0;

        std::map<int, bool> lidar_data_update_map_;
        std::map<int, std::vector<autol_msgs::msg::AutolPacket>> lidar_data_map_;
        std::map<int, int> point_count_by_lidar_map_;
        rclcpp::Node *node_;
        bool is_previous_frame_processing_;
    };
}
#endif