#include "shape_detector.hpp"
#include "shape_detector_utils.hpp"


//--Shape Detector Node  -------------------------------------------------------------------
ShapeDetectorNode::ShapeDetectorNode() : Node("shape_detector")
{
    image_subscriber_ = image_transport::create_subscription(
        this, "/camera/image_raw",
        std::bind(&ShapeDetectorNode::image_callback, this, std::placeholders::_1),
        "raw");

    // Creates a window to show the shape detection results 
    cv::namedWindow("Shape Detection", cv::WINDOW_AUTOSIZE);
    output_file_.open("detected_shapes.txt", std::ios::out | std::ios::app);
    if (!output_file_.is_open())
    {
        // Error Handling Message text log file cannot be opened
        RCLCPP_ERROR(this->get_logger(), "Failed to open output file.");
    }
}
//-- Shape Detector Destructor -------------------------------------------------------------
ShapeDetectorNode::~ShapeDetectorNode()
{
    if (output_file_.is_open())
    {
        // Closes the file once open for debugging somehow doesn't work in practice. 
        output_file_.close(); 
    }
}

//-- Image Callback Function for the Shape Detector Node -----------------------------------
void ShapeDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {
        // Convert the image to an OpenCV image. 
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat gray, blurred, thresh, edged;

        // Pre Processing the image with thresholding 
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        cv::adaptiveThreshold(blurred, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
        cv::Canny(thresh, edged, 50, 150);

        // Finding Contours in the image
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(edged.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // Minimum Area to consider and a flag for shape detection 
        const double min_contour_area = 1000.0;
        bool shapes_found = false;

        // Loop through the contours detected
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double contour_area = cv::contourArea(contours[i]);
            if (contour_area < min_contour_area)
            {
                // Skip any small little shape "noise" found in the image
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

        // Display the scanned image with the detected shape with a small timeframe
        cv::imshow("Shape Detection", frame);
        cv::waitKey(1);

        // Closes the subscriber
        image_subscriber_.shutdown();
        RCLCPP_INFO(this->get_logger(), "Image processed, subscription closed.");
    }
    catch (const cv_bridge::Exception &e)
    {
        // Error Handler for the cv_bridge
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}