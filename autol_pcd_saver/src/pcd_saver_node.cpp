#include <rclcpp/rclcpp.hpp>
#include "pcd_saver.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  std::shared_ptr<PcdSaver> pcd_save_node = std::make_shared<PcdSaver>(options);
  return 0;
}

