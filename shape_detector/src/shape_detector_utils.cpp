// shape_detector_utils.cpp

#include "shape_detector_utils.hpp"

// Function to detect the shape of an object scanned via the camera
size_t detect_shape(const std::vector<cv::Point>& contour, std::string &shape)
{
    // Calculate the length of the contour
    double peri = cv::arcLength(contour, true);

    // Approximate the contour with fewer points to simplify the polygon. 
    std::vector<cv::Point> approx;

    // Set the approx threshold for the edge and corner detection
    cv::approxPolyDP(contour, approx, 0.0035 * peri, true); 

    // Number of Verticies. 
    size_t vertices = approx.size();

    //--Shape detection switch case. 
    if (vertices == static_cast<size_t>(ShapeType::TRIANGLE))
    {
        shape = "Triangle";
    }
    else if (vertices == static_cast<size_t>(ShapeType::SQUARE))
    {
        double aspect_ratio = (double)cv::boundingRect(approx).width / cv::boundingRect(approx).height;
        shape = (aspect_ratio >= 0.95 && aspect_ratio <= 1.05) ? "Square" : "Rectangle";
    }
    else if (vertices == static_cast<size_t>(ShapeType::PENTAGON))
    {
        shape = "Pentagon";
    }
    else if (vertices == static_cast<size_t>(ShapeType::HEXAGON))
    {
        shape = "Hexagon";
    }
    else if (vertices == static_cast<size_t>(ShapeType::HEPTAGON))
    {
        shape = "Heptagon";
    }
    else if (vertices == static_cast<size_t>(ShapeType::OCTAGON))
    {
        shape = "Octagon";
    }
    else if (vertices == static_cast<size_t>(ShapeType::NONAGON))
    {
        shape = "Nonagon";
    }
    else if (vertices == static_cast<size_t>(ShapeType::DECAGON))
    {
        shape = "Decagon";
    }
    else if (vertices >= static_cast<size_t>(ShapeType::UNIDENTIFIED) && vertices <= 14)
    {
        shape = "Unidentified Shape";
    }
    else if (vertices >= static_cast<size_t>(ShapeType::CIRCLE))
    {
        shape = "Circle";
    }
    return vertices;
}

// Function to detect the colour of an object scanned via the camera
std::string detect_color(const cv::Mat &frame, const std::vector<cv::Point> &contour)
{
    // Create a mask for the contour area
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), -1);

    // Determine the mean colour within the contour. 
    cv::Scalar mean_bgr = cv::mean(frame, mask);

    // Convert the colour from BGR to HSV
    cv::Mat bgr(1, 1, CV_8UC3, mean_bgr);
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    int hue = hsv.at<cv::Vec3b>(0, 0)[0];

    //-- Colour Identification switch case
    if (hue >= static_cast<int>(ColorRange::RED_MIN) && hue <= static_cast<int>(ColorRange::RED_MAX))
    {
        return "Red";
    }
    else if (hue >= static_cast<int>(ColorRange::ORANGE_MIN) && hue <= static_cast<int>(ColorRange::ORANGE_MAX))
    {
        return "Orange";
    }
    else if (hue >= static_cast<int>(ColorRange::YELLOW_MIN) && hue <= static_cast<int>(ColorRange::YELLOW_MAX))
    {
        return "Yellow";
    }
    else if (hue >= static_cast<int>(ColorRange::GREEN_MIN) && hue <= static_cast<int>(ColorRange::GREEN_MAX))
    {
        return "Green";
    }
    else if (hue >= static_cast<int>(ColorRange::BLUE_MIN) && hue <= static_cast<int>(ColorRange::BLUE_MAX))
    {
        return "Blue";
    }
    else if (hue >= static_cast<int>(ColorRange::PURPLE_MIN) && hue <= static_cast<int>(ColorRange::PURPLE_MAX))
    {
        return "Purple";
    }
    else if (hue >= static_cast<int>(ColorRange::PINK_MIN) && hue <= static_cast<int>(ColorRange::PINK_MAX))
    {
        return "Pink";
    }
    else if (hue >= static_cast<int>(ColorRange::WHITE_MIN) && hue <= static_cast<int>(ColorRange::WHITE_MAX))
    {
        return "White";
    }
    else
    {
        return "Unknown Color";
    }
}
