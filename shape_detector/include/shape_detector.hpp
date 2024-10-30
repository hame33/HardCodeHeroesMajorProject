#ifndef SHAPE_DETECTOR_HPP
#define SHAPE_DETECTOR_HPP

// --- Includes ---
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <memory>
#include "shape_detector_utils.hpp"  // Utility header for shape and color detection

// --- ShapeDetectorNode Interface ---
// The ShapeDetectorNode class is responsible for detecting shapes in incoming image data.
// This class subscribes to image data, processes it to detect specific shapes and colors, 
// and outputs the detected information to a file.
class ShapeDetectorNode : public rclcpp::Node
{
public:
  // Constructor - Initializes the ShapeDetectorNode
  ShapeDetectorNode();

  // Destructor
  ~ShapeDetectorNode();

private:
  // --- Components ---
  std::shared_ptr<ShapeDetectorUtils> shape_detector_utils_;  // Utility for shape and color detection

  // --- Subscribers ---
  image_transport::Subscriber image_subscriber_;  // Subscriber for receiving images

  // --- Callback Methods ---
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);  // Processes incoming image data

  // --- Data ---
  std::ofstream output_file_;  // File stream for logging detected shape information
};

#endif  // SHAPE_DETECTOR_HPP
