// main.cpp

#include "InventoryWaypointsNode.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  std::cout << "Main" << std::endl;
  rclcpp::init(argc, argv);
  std::cout << "rclcpp initialise" << std::endl;
  rclcpp::spin(std::make_shared<InventoryWaypointsNode>());
  rclcpp::shutdown();

  return 0;
}

