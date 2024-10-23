#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

class ShapeDetect {
public:
    // Constructor that accepts the image path
    ShapeDetect(const string& path) : imagePath(path) {}

    // Function to execute shape detection
    void detectAndDrawShapes() {
        // Read the input image
        Mat inputImage = imread(imagePath);
        if (inputImage.empty()) {
            cerr << "Error: Could not open or find the image!" << endl;
            return;
        }

        // Resize the image
        Mat resizedImage;
        resize(inputImage, resizedImage, Size(500, 500));

        // Convert the image to grayscale
        Mat gray;
        cvtColor(resizedImage, gray, COLOR_BGR2GRAY);

        // Apply Gaussian Blur to reduce noise
        Mat blur;
        GaussianBlur(gray, blur, Size(5, 5), 0);

        // Use Canny edge detection
        Mat edges;
        Canny(blur, edges, 50, 200);

        // Morphological operations
        Mat morphedEdges;
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(edges, morphedEdges, MORPH_CLOSE, kernel);

        // Find contours in the edges
        vector<vector<Point>> contours;
        findContours(morphedEdges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Check if any contours were found
        if (contours.empty()) {
            cerr << "No contours found!" << endl;
            return; // Exit early if no contours
        } else {
            cerr << contours.size() << endl;
        }

        // Create a copy of the input image to draw on
        Mat outputImage = resizedImage.clone();

        // Loop through the contours and draw them
        for (size_t i = 0; i < contours.size(); i++) {
            // Approximate the contour to a polygon
            vector<Point> approx;
            double area = contourArea(contours[i]);
            if (area > 100) { // Filter out small areas
                approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);

                // Filter points based on minimum distance
                vector<Point> filteredApprox = filterClosePoints(approx, 5.0); // Adjust distance threshold as needed

                // Determine the shape based on the number of vertices
                string shape;
                if (filteredApprox.size() == 3 && isContourConvex(filteredApprox)) {
                    shape = "Triangle";
                } else if (filteredApprox.size() == 4) {
                    // Check if it's a square or rectangle
                    Rect boundingBox = boundingRect(filteredApprox);
                    double aspectRatio = (double)boundingBox.width / boundingBox.height;
                    shape = (aspectRatio >= 0.95 && aspectRatio <= 1.05) ? "Square" : "Rectangle";
                } else if (filteredApprox.size() == 5) {
                    shape = "Pentagon";
                } else if (filteredApprox.size() == 6) {
                    shape = "Hexagon";
                } else if (filteredApprox.size() > 6) {
                    shape = "Circle";        
                } else {
                    shape = "Unknown";
                }

                // Draw the contours
                drawContours(outputImage, contours, static_cast<int>(i), Scalar(0, 255, 0), 2);

                // Calculate the position for the text in the top right corner
                Point textPosition(outputImage.cols - 100, 15); // Adjust position as needed
                
                // Put the shape name on the image
                putText(outputImage, shape, textPosition, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);
            }
        }

        // Save the output image
        imwrite("output.jpg", outputImage);
        
        // Display the original image with overlays
        imshow("Shape Detection", outputImage);
        waitKey(0); // Wait for a key press to close the window
    }

private:
    string imagePath; // Path to the image

    // Function to calculate the distance between two points
    double distance(const Point& a, const Point& b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    // Function to filter points based on minimum distance
    vector<Point> filterClosePoints(const vector<Point>& points, double minDistance) {
        vector<Point> filteredPoints;
        for (const auto& point : points) {
            bool isFarEnough = true;
            for (const auto& filteredPoint : filteredPoints) {
                if (distance(point, filteredPoint) < minDistance) {
                    isFarEnough = false;
                    break;
                }
            }
            if (isFarEnough) {
                filteredPoints.push_back(point);
            }
        }
        return filteredPoints;
    }
};


class ColorDetector {
public:
    // Constructor
    ColorDetector(const string& imagePath) {
        // Define color ranges for red, green, and blue
        redRanges.push_back({Scalar(0, 100, 100), Scalar(10, 255, 255)});
        redRanges.push_back({Scalar(160, 100, 100), Scalar(180, 255, 255)});
        greenRange = {Scalar(40, 100, 100), Scalar(80, 255, 255)};
        blueRange = {Scalar(100, 100, 100), Scalar(140, 255, 255)};

        identifyColor(imagePath);
    }

    // Method to identify the dominant color
    void identifyColor(const string& imagePath) {
        // Read the input image
        Mat image = imread(imagePath);
        if (image.empty()) {
            cerr << "Error: Could not open or find the image!" << endl;
            return;
        }

        Mat hsvImage;
        cvtColor(image, hsvImage, COLOR_BGR2HSV);

        Mat redMask = createRedMask(hsvImage);
        Mat greenMask, blueMask;
        inRange(hsvImage, greenRange.lower, greenRange.upper, greenMask);
        inRange(hsvImage, blueRange.lower, blueRange.upper, blueMask);

        // Count non-zero pixels in masks
        int redCount = countNonZero(redMask);
        int greenCount = countNonZero(greenMask);
        int blueCount = countNonZero(blueMask);

        // Identify dominant color
        if (redCount > greenCount && redCount > blueCount) {
            detectedColour = "red";
        } else if (greenCount > redCount && greenCount > blueCount) {
            detectedColour = "green";
        } else if (blueCount > redCount && blueCount > greenCount) {
            detectedColour = "blue";
        } else {
            detectedColour = "unknown";
        }
    }

    // Method to output the detected color
    void printColor() {
        cout << "The detected color is: " << detectedColour << endl;
    }

private:
    struct ColorRange {
        Scalar lower;
        Scalar upper;
    };

    vector<ColorRange> redRanges;
    ColorRange greenRange;
    ColorRange blueRange;

    string detectedColour;

    Mat createRedMask(const Mat& hsvImage) {
        Mat redMask1, redMask2, combinedRedMask;
        inRange(hsvImage, redRanges[0].lower, redRanges[0].upper, redMask1);
        inRange(hsvImage, redRanges[1].lower, redRanges[1].upper, redMask2);
        combinedRedMask = redMask1 | redMask2;
        return combinedRedMask;
    }
};

// int main() {
//     // Hard-coded image path
//     const string imagePath = "red_circle.jpeg"; // Replace with your actual image filename

//     // Create a ShapeDetect object
//     ShapeDetect shapeDetector(imagePath);

//     // Execute shape detection
//     shapeDetector.detectAndDrawShapes();

//     // Create an instance of ColorDetector
//     ColorDetector detector(imagePath);
//     detector.printColor();

//     return 0;
// }

int main(int argc, char** argv) {
    // Check for command-line argument
    if (argc < 2) {
        cerr << "Usage: ./ImageDetect <path_to_image>" << endl;
        return 1; // Exit if no image path is provided
    }

    // Get the image path from command-line arguments
    const string imagePath = argv[1];

    // Create a ShapeDetect object
    ShapeDetect shapeDetector(imagePath);

    // Execute shape detection
    shapeDetector.detectAndDrawShapes();

    // Create an instance of ColorDetector
    ColorDetector detector(imagePath);
    detector.printColor();

    return 0;
}