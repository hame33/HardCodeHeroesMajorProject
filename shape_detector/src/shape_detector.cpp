// --- Includes ---
#include "shape_detector.hpp"

// --- ShapeDetectorNode Implementation ---

// --- Constructor ---
ShapeDetectorNode::ShapeDetectorNode() : Node("shape_detector")
{
  // Initialize shape detector utilities
  shape_detector_utils_ = std::make_shared<ShapeDetectorUtils>();

  // Subscribe to the camera's raw image topic
  image_subscriber_ = image_transport::create_subscription(
    this, "/camera/image_raw",
    std::bind(&ShapeDetectorNode::image_callback, this, std::placeholders::_1),
    "raw");

  // Initialize output display and file for detected shapes
  cv::namedWindow("Shape Detection", cv::WINDOW_AUTOSIZE);
  output_file_.open("detected_shapes.txt", std::ios::out | std::ios::app);

  // Error handling if file fails to open
  if (!output_file_.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open output file.");
  }
}

// --- Destructor ---
ShapeDetectorNode::~ShapeDetectorNode()
{
  // Close the output file if it is open
  if (output_file_.is_open())
  {
    output_file_.close();
  }
}

// --- image_callback ---
void ShapeDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
  try
  {
    // Convert the incoming ROS image message to an OpenCV image format
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat gray, blurred, thresh, edged;

    // Convert to grayscale, blur, threshold, and detect edges in the image
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::adaptiveThreshold(blurred, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
    cv::Canny(thresh, edged, 50, 150);

    // Find contours in the processed image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edged.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    const double min_contour_area = 1000.0;  // Minimum area for a contour to be considered
    bool shapes_found = false;

    // Iterate over each detected contour
    for (size_t i = 0; i < contours.size(); ++i)
    {
      double contour_area = cv::contourArea(contours[i]);
      if (contour_area < min_contour_area)
      {
        // Skip small contours that don't meet the area threshold
        continue;
      }

      shapes_found = true;

      // Detect shape type and number of vertices
      std::string shape;
      size_t vertices = shape_detector_utils_->detect_shape(contours[i], shape);

      // For circles, force vertices count to zero
      if (shape == "Circle")
      {
        vertices = 0;
      }

      // Draw detected contour and label the shape in the image
      cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contours[i]}, -1, cv::Scalar(0, 255, 0), 2);
      cv::putText(frame, shape, contours[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
      std::string color = shape_detector_utils_->detect_color(frame, contours[i]);

      // Log detected shape details
      RCLCPP_INFO(this->get_logger(), "Detected Shape: %s, Vertices: %zu, Color: %s", shape.c_str(), vertices, color.c_str());

      // Write detected shape details to output file
      if (output_file_.is_open())
      {
        output_file_ << shape << " " << vertices << " " << color << "\n";
      }

      // Terminate after detecting the first valid shape
      RCLCPP_INFO(this->get_logger(), "Exiting after first detection.");
      rclcpp::shutdown();
      return;
    }

    // If no shapes were found in the image, log and shut down
    if (!shapes_found)
    {
      RCLCPP_INFO(this->get_logger(), "No shapes found in the image. Exiting.");
      rclcpp::shutdown();
      return;
    }

    // Display the processed image with detected shapes
    cv::imshow("Shape Detection", frame);
    cv::waitKey(1);

    // Close the subscription after processing the image
    image_subscriber_.shutdown();
    RCLCPP_INFO(this->get_logger(), "Image processed, subscription closed.");
  }
  catch (const cv_bridge::Exception &e)
  {
    // Handle cv_bridge conversion exceptions
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}
