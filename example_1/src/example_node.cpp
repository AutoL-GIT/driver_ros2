#include <rclcpp/rclcpp.hpp>
#include "example_1/RcvPcd.h"

using namespace example;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  std::shared_ptr<RcvPcd> example_node = std::make_shared<RcvPcd>(options);
  example_node->DoSomthing();
  return 0;
}

