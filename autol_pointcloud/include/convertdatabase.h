#ifndef AUTOL_ROS_CONVERT_DATABASE_H_
#define AUTOL_ROS_CONVERT_DATABASE_H_

#include "define.h"
namespace autol_data
{
    class ConvertDataBase
    {
    public:
        void SetRosNode(rclcpp::Node *node) { node_ = node; }
        ConvertDataBase(rclcpp::Node *node, const double max_range, const double min_range, const std::string &target_frame,
                        const std::string &fixed_frame, const unsigned int init_width, const unsigned int init_height,
                        const bool is_dense, const unsigned int scans_per_packet,
                        std::shared_ptr<tf2_ros::Buffer> &tf_ptr, int fields, ...)
            : config_(max_range, min_range, target_frame, fixed_frame, init_width, init_height, is_dense, scans_per_packet), tf_ptr(tf_ptr)
        {
            va_list vl;
            cloud.fields.clear();
            cloud.fields.reserve(fields);
            va_start(vl, fields);
            int offset = 0;
            for (int i = 0; i < fields; ++i)
            {
                std::string name(va_arg(vl, char *));
                int count(va_arg(vl, int));
                int datatype(va_arg(vl, int));
                offset = addPointField(cloud, name, count, datatype, offset);
            }
            va_end(vl);
            cloud.point_step = offset;
            cloud.row_step = init_width * cloud.point_step;
            if (config_.transform && !tf_ptr)
            {
                tf_ptr = std::make_shared<tf2_ros::Buffer>(node->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_ptr);
            }
        }

        virtual void Setup(const autol_msgs::msg::AutolFrame::ConstPtr &frameMsg)
        {
            cloud.header = frameMsg->header;

            cloud.data.resize(frameMsg->packets.size() * config_.scans_per_packet * cloud.point_step);
            cloud.width = config_.init_width;
            cloud.height = config_.init_height;
            cloud.is_dense = static_cast<uint8_t>(config_.is_dense);
        }
        sensor_msgs::msg::PointCloud2 cloud;
        sensor_msgs::msg::PointCloud2 resizeData(rclcpp::Node *node);

        struct Config
        {
            double max_range;         ///< maximum range to publish
            double min_range;         ///< minimum range to publish
            std::string target_frame; ///< target frame to transform a point
            std::string fixed_frame;  ///< fixed frame used for transform
            unsigned int init_width;
            unsigned int init_height;
            bool is_dense;
            unsigned int scans_per_packet;
            bool transform; ///< enable / disable transform points

            Config(double max_range, double min_range, std::string target_frame, std::string fixed_frame,
                   unsigned int init_width, unsigned int init_height, bool is_dense, unsigned int scans_per_packet)
                : max_range(max_range), min_range(min_range), target_frame(target_frame), fixed_frame(fixed_frame), transform(fixed_frame != target_frame), init_width(init_width), init_height(init_height), is_dense(is_dense), scans_per_packet(scans_per_packet)
            {
            }
        };

        virtual void insertPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth, const float distance, const float intensity, const float time) = 0;

    protected:
        Config config_;
        std::shared_ptr<tf2_ros::Buffer> tf_ptr;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    private:
        rclcpp::Node *node_;
    };
}
#endif
