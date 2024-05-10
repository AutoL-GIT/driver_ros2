#include "driver.hpp"

std::mutex g_mtx;
std::condition_variable g_cv;

void SigintHandler(int sig)
{
  cout << "Shut Down ROS Driver\n";
  g_cv.notify_all();
}

int main(int argc, char **argv) // Node Main Function

{
  signal(SIGINT, SigintHandler); //< bind ctrl+c signal with the sigHandler function
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  std::shared_ptr<AutolDriver> autol_driver = std::make_shared<AutolDriver>(options);
  autol_driver->Init(); //Init Configuration 
  autol_driver->Start(); // Driver Start

  std::unique_lock<std::mutex> lck(g_mtx);
  g_cv.wait(lck);
  
  rclcpp::shutdown();
  RCLCPP_INFO(autol_driver->get_logger(), "Close the AutoL ROS Driver");
  return 0;
}
