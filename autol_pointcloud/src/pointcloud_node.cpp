#include "convert.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    auto convert_node = std::make_shared<autol_pointcloud::Convert>(options);
    rclcpp::spin(convert_node);
    RCLCPP_INFO(convert_node->get_logger(), "Shutdown Point Cloud Node");
    rclcpp::shutdown();
    return 0;
}
