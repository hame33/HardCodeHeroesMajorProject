#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include "shape_detector.hpp"

ShapeDetectorNode::ShapeDetectorNode() : Node("shape_detector")
{
    // Subscribe to the camera feed
    image_subscriber_ = image_transport::create_subscription(
        this, "/camera/image_raw",
        std::bind(&ShapeDetectorNode::image_callback, this, std::placeholders::_1),
        "raw");

    // Initialize OpenCV window
    cv::namedWindow("Shape Detection", cv::WINDOW_AUTOSIZE);

    // Open file for appending detected shapes
    output_file_.open("detected_shapes.txt", std::ios::out | std::ios::app);
    if (!output_file_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open output file.");
    }
}

ShapeDetectorNode::~ShapeDetectorNode()
{
    // Close the output file when the node is destroyed
    if (output_file_.is_open())
    {
        output_file_.close();
    }
}

void ShapeDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {
        // Convert ROS image message to OpenCV image
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat gray, blurred, thresh, edged;

        // Convert to grayscale
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Apply Gaussian blur to reduce noise
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

        // Apply adaptive thresholding to detect shapes dynamically
        cv::adaptiveThreshold(blurred, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);

        // Edge detection using Canny
        cv::Canny(thresh, edged, 50, 150);

        // Find contours from the edge-detected image
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(edged.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Filter out small contours
        const double min_contour_area = 1000.0;
        bool shapes_found = false;

        for (size_t i = 0; i < contours.size(); ++i)
        {
            double contour_area = cv::contourArea(contours[i]);
            if (contour_area < min_contour_area)
                continue;

            shapes_found = true;

            // Approximate the contour to a polygon
            double peri = cv::arcLength(contours[i], true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[i], approx, 0.04 * peri, true);

            // Ignore rectangular background shapes
            if (approx.size() == 4)
            {
                cv::Rect bounding_box = cv::boundingRect(approx);
                double aspect_ratio = static_cast<double>(bounding_box.width) / bounding_box.height;

                if (aspect_ratio >= 0.7 && aspect_ratio <= 1.6)
                {
                    RCLCPP_INFO(this->get_logger(), "Ignoring rectangular background shape.");
                    continue;
                }
            }

            // Determine shape based on number of vertices
            std::string shape;
            size_t vertices = approx.size();

            if (vertices == 3)
                shape = "Triangle";
            else if (vertices == 4)
            {
                double aspect_ratio = (double)cv::boundingRect(approx).width / cv::boundingRect(approx).height;
                shape = (aspect_ratio >= 0.95 && aspect_ratio <= 1.05) ? "Square" : "Rectangle";
            }
            else if (vertices == 5)
                shape = "Pentagon";
            else if (vertices == 6)
                shape = "Hexagon";
            else if (vertices == 7)
                shape = "Heptagon";
            else if (vertices == 8)
                shape = "Octagon";
            else
                shape = "Unknown";

            // Detect color of the shape
            std::string color = detect_color(frame, contours[i]);

            // Log detected shape information
            RCLCPP_INFO(this->get_logger(), "Detected Shape: %s, Vertices: %zu, Color: %s", shape.c_str(), vertices, color.c_str());

            // Write detected shape information to file
            if (output_file_.is_open())
            {
                output_file_ << shape << ", " << vertices << ", " << color << "\n";
            }

            // Draw contours and label shapes
            cv::drawContours(frame, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, shape + " - " + color, approx[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

            // Show the image for 3 seconds
            cv::imshow("Shape Detection", frame);
            cv::waitKey(3000); 

            // Exit after the first valid shape
            RCLCPP_INFO(this->get_logger(), "Exiting after first detection.");
            rclcpp::shutdown();
            return; 
        }

        // If no shapes found, log it
        if (!shapes_found)
        {
            RCLCPP_INFO(this->get_logger(), "No shapes found in the image.");
            cv::imshow("Shape Detection", frame);
            cv::waitKey(3000);
            rclcpp::shutdown();
            return; 
        }

        // Show the image and allow OpenCV to process events
        cv::imshow("Shape Detection", frame);
        cv::waitKey(1); 

        // Unsubscribe after processing one image
        image_subscriber_.shutdown();
        RCLCPP_INFO(this->get_logger(), "Image processed, subscription closed.");
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

std::string ShapeDetectorNode::detect_color(const cv::Mat &frame, const std::vector<cv::Point> &contour)
{
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), -1);
    cv::Scalar mean_bgr = cv::mean(frame, mask);

    // Convert BGR to HSV for accurate color detection
    cv::Mat bgr(1, 1, CV_8UC3, mean_bgr);
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // Get the Hue value
    int hue = hsv.at<cv::Vec3b>(0, 0)[0];

    // Determine color based on Hue value
    if (hue >= 0 && hue <= 10)
        return "Red";
    else if (hue >= 11 && hue <= 25)
        return "Orange";
    else if (hue >= 26 && hue <= 34)
        return "Yellow";
    else if (hue >= 35 && hue <= 85)
        return "Green";
    else if (hue >= 86 && hue <= 125)
        return "Blue";
    else if (hue >= 126 && hue <= 160)
        return "Purple";
    else if (hue >= 161 && hue <= 180)
        return "Pink"; // Add Pink range
    else if (mean_bgr[0] > 200 && mean_bgr[1] > 200 && mean_bgr[2] > 200)
        return "White"; // Detect white based on BGR mean values
    else
        return "Unknown Color";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Check for command line arguments to clear the output file
    bool clear_file = false;
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--clear")
        {
            clear_file = true;
            break;
        }
    }

    // Clear the output file if the flag is set and terminate without running the scan
    if (clear_file)
    {
        std::ofstream output_file("detected_shapes.txt", std::ios::out | std::ios::trunc);
        output_file.close();
        RCLCPP_INFO(rclcpp::get_logger("ShapeDetectorNode"), "Output file cleared.");
        rclcpp::shutdown(); // Shutdown the ROS node
        return 0; // Exit the program
    }

    auto node = std::make_shared<ShapeDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
