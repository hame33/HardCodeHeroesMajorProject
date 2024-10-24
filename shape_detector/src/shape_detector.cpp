#include "shape_detector.hpp"
#include "shape_detector_utils.hpp"

ShapeDetectorNode::ShapeDetectorNode() : Node("shape_detector")
{
    image_subscriber_ = image_transport::create_subscription(
        this, "/camera/image_raw",
        std::bind(&ShapeDetectorNode::image_callback, this, std::placeholders::_1),
        "raw");

    cv::namedWindow("Shape Detection", cv::WINDOW_AUTOSIZE);
    output_file_.open("detected_shapes.txt", std::ios::out | std::ios::app);
    if (!output_file_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open output file.");
    }
}

ShapeDetectorNode::~ShapeDetectorNode()
{
    if (output_file_.is_open())
    {
        output_file_.close();
    }
}

void ShapeDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat gray, blurred, thresh, edged;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        cv::adaptiveThreshold(blurred, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
        cv::Canny(thresh, edged, 50, 150);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(edged.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        const double min_contour_area = 1000.0;
        bool shapes_found = false;

        for (size_t i = 0; i < contours.size(); ++i)
        {
            double contour_area = cv::contourArea(contours[i]);
            if (contour_area < min_contour_area)
            {
                continue;
            }

            shapes_found = true;

            // Shape detection logic
            std::string shape;
            size_t vertices = detect_shape(contours[i], shape);

            // Check if the detected shape is a circle
            if (shape == "Circle")
            {
                vertices = 0; // Force vertices to 0 for circles
            }

            // Draw contours and label shapes
            cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contours[i]}, -1, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, shape, contours[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            std::string color = detect_color(frame, contours[i]);

            // Log detected shape information
            RCLCPP_INFO(this->get_logger(), "Detected Shape: %s, Vertices: %zu, Color: %s", shape.c_str(), vertices, color.c_str());

            if (output_file_.is_open())
            {
                output_file_ << shape << " " << vertices << " " << color << "\n";
            }

            // Exit after the first valid shape
            RCLCPP_INFO(this->get_logger(), "Exiting after first detection.");
            rclcpp::shutdown();
            return;
        }

        if (!shapes_found)
        {
            RCLCPP_INFO(this->get_logger(), "No shapes found in the image. Exiting.");
            rclcpp::shutdown();  // Terminates the node when no shapes are found
            return;
        }

        cv::imshow("Shape Detection", frame);
        cv::waitKey(1);
        image_subscriber_.shutdown();
        RCLCPP_INFO(this->get_logger(), "Image processed, subscription closed.");
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
