#ifndef SHAPE_DETECTOR_HPP
#define SHAPE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>

class ShapeDetectorNode : public rclcpp::Node
{
public:
    ShapeDetectorNode();
    ~ShapeDetectorNode();

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    std::string detect_color(const cv::Mat &frame, const std::vector<cv::Point> &contour);

    image_transport::Subscriber image_subscriber_;
    std::ofstream output_file_;
};

#endif // SHAPE_DETECTOR_HPP

