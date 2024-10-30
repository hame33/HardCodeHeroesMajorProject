#ifndef SHAPE_DETECTOR_UTILS_HPP
#define SHAPE_DETECTOR_UTILS_HPP

// --- Includes ---
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>

// --- ShapeDetectorUtils Class Interface ---
// The ShapeDetectorUtils class provides utilities for detecting shapes and colors 
// within an image. It includes methods for determining the type of shape based 
// on contour points and detecting the color based on HSV values.

class ShapeDetectorUtils 
{
public:
  // Constructor - Default constructor for ShapeDetectorUtils
  ShapeDetectorUtils();

  // --- ShapeDetectorUtils Functions ---

  // detect_shape - Detects shape type from contour and outputs the shape's name
  size_t detect_shape(const std::vector<cv::Point>& contour, std::string &shape);

  // detect_color - Detects color based on HSV values from the frame and contour area
  std::string detect_color(const cv::Mat &frame, const std::vector<cv::Point> &contour);

  // --- Enums ---
  // Enumeration defining various shape types by vertex count
  enum class ShapeType 
  {
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

  // Enumeration defining HSV ranges for color detection
  enum class ColorRange 
  {
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
};

#endif  // SHAPE_DETECTOR_UTILS_HPP
