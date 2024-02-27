#include "convert.h"

namespace autol_pointcloud
{
    int32_t cnt = 0;
    Convert::Convert(const rclcpp::NodeOptions &node_options)
        : Node("autol_pointcloud_node", node_options), raw_data_(new autol_data::RawData(this))
    {
        RCLCPP_INFO(this->get_logger(), "Convert()");
        this->declare_parameter("calibration", rclcpp::ParameterValue(false));
        this->get_parameter("calibration", is_slam_active_);
        raw_data_->Setup();
        convert_data_ = std::shared_ptr<autol_data::PointcloudXYZ>(new autol_data::PointcloudXYZ(this, 0, 0, "map", "map", raw_data_->pointsPerPacket()));
        convert_data_->SetRosNode(this);
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        pub_autol_pointcloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("autol_pointcloud", 1);
        sub_frame_ = this->create_subscription<autol_msgs::msg::AutolFrame>(
            "autol_frame_data", qos_profile, std::bind(&Convert::frameData_cb, this, std::placeholders::_1));
        rclcpp::spin(this->get_node_base_interface());
        is_previous_frame_processing_ = false;
    }

    void Convert::frameData_cb(const autol_msgs::msg::AutolFrame::ConstPtr &frameMsg)
    {
        cnt++;
        if (is_previous_frame_processing_ == true)
        {
            RCLCPP_INFO(this->get_logger(), "Prevous Frame Processsing");
            return;
        }
        is_previous_frame_processing_ = true;

        lidar_data_update_map_[frameMsg->lidar_index] = true;
        for (int i = 0; i < frameMsg->packets.size(); ++i)
            lidar_data_map_[frameMsg->lidar_index].push_back(frameMsg->packets[i]);

        bool is_all_update = true;

        for (auto &lidar : lidar_data_update_map_)
        {
            if (lidar.second == false)
            {
                is_all_update = false;
                break;
            }
        }

        if (is_all_update)
        {
            auto merged_frame = std::make_shared<autol_msgs::msg::AutolFrame>();

            int packet_size = 0;
            for (auto &lidar : lidar_data_map_)
            {
                packet_size += lidar.second.size();
            }

            merged_frame->packets.reserve(packet_size);

            int point_accum_count = 0;

            for (auto &lidar : lidar_data_map_)
            {
                merged_frame->packets.insert(merged_frame->packets.end(), lidar.second.begin(), lidar.second.end());
                point_accum_count += lidar.second.size();
                point_count_by_lidar_map_[lidar.first] = point_accum_count;
            }

            for (auto &lidar : lidar_data_update_map_)
                lidar.second = false;
            for (auto &lidar : lidar_data_map_)
                lidar.second.clear();

            convert_data_->Setup(merged_frame);
            for (size_t i = 0; i < merged_frame->packets.size(); ++i)
            {
                int lidar_index_of_point = 0;
                std::map<int, int>::reverse_iterator ritr;
                for (ritr = point_count_by_lidar_map_.rbegin(); ritr != point_count_by_lidar_map_.rend(); ++ritr)
                {
                    if (i < ritr->second)
                        lidar_index_of_point = ritr->first;
                }
                raw_data_->ConvertFromRaw(merged_frame->packets[i], *convert_data_, is_slam_active_, lidar_index_of_point);
            }
            auto colud_msg = convert_data_->resizeData(this);
            pub_autol_pointcloud_->publish(colud_msg);
        }
        else
        {
        }

        is_previous_frame_processing_ = false;
    }
}
