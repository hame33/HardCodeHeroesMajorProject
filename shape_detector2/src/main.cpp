#include <rclcpp/rclcpp.hpp>
#include "shape_detector.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto shape_detector_node = std::make_shared<ShapeDetectorNode>();
    rclcpp::spin(shape_detector_node);
    rclcpp::shutdown();
    return 0;
}
