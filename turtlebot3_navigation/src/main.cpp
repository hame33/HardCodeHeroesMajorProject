// main.cpp

#include "ShopNavAndDriveNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShopNavAndDriveNode>());
  rclcpp::shutdown();

  return 0;
}



