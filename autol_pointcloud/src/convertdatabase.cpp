#include "convertdatabase.h"

namespace autol_data
{
    sensor_msgs::msg::PointCloud2 ConvertDataBase::resizeData(rclcpp::Node *node)
    {
        cloud.data.resize(cloud.point_step * cloud.width * cloud.height);
        if (!config_.target_frame.empty())
        {
            cloud.header.frame_id = config_.target_frame;
        }
        return cloud;
    }
}
