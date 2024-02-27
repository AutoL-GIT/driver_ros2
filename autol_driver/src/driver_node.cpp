#include "define.h"
#include "driver.h"

using namespace autol_driver;
std::shared_ptr<AutolDriver> driver_node;

void SigintHandler(int sig)
{
  RCLCPP_INFO(driver_node->get_logger(), "Stop AuoL ROS Driver");
  rclcpp::shutdown();
}

int main(int argc, char **argv) // Node Main Function
{
  signal(SIGINT, SigintHandler); //< bind ctrl+c signal with the sigHandler function
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  driver_node = std::make_shared<AutolDriver>(options);
  driver_node->StartRecvPacket();
  RCLCPP_INFO(driver_node->get_logger(), "closing");
  rclcpp::shutdown();
  return 0;
}
