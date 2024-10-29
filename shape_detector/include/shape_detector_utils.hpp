// shape_detector_utils.hpp

#ifndef SHAPE_DETECTOR_UTILS_HPP
#define SHAPE_DETECTOR_UTILS_HPP

#include <vector>
#include <opencv2/opencv.hpp>

// Enum for Shape Types
enum class ShapeType {
    TRIANGLE = 3,
    SQUARE = 4,
    RECTANGLE = 5,
    PENTAGON = 5,
    HEXAGON = 6,
    HEPTAGON = 7,
    OCTAGON = 8,
    NONAGON = 9,
    DECAGON = 10,
    UNIDENTIFIED = 11,
    CIRCLE = 14
};

// Enum for Color Ranges
enum class ColorRange {
    RED_MIN = 0,
    RED_MAX = 10,
    ORANGE_MIN = 11,
    ORANGE_MAX = 20,
    YELLOW_MIN = 21,
    YELLOW_MAX = 34,
    GREEN_MIN = 35,
    GREEN_MAX = 85,
    BLUE_MIN = 86,
    BLUE_MAX = 125,
    PURPLE_MIN = 126,
    PURPLE_MAX = 160,
    PINK_MIN = 161,
    PINK_MAX = 180,
    WHITE_MIN = 181,
    WHITE_MAX = 200
};

// Function declarations
size_t detect_shape(const std::vector<cv::Point>& contour, std::string &shape);
std::string detect_color(const cv::Mat &frame, const std::vector<cv::Point> &contour);

#endif // SHAPE_DETECTOR_UTILS_HPP
