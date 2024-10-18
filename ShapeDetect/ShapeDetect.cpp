#include <opencv2/opencv.hpp>
#include <iostream>

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

        // Convert the image to grayscale
        Mat gray;
        cvtColor(inputImage, gray, COLOR_BGR2GRAY);

        // Apply Gaussian Blur to reduce noise
        GaussianBlur(gray, gray, Size(5, 5), 0);

        // Use Canny edge detection
        Mat edges;
        Canny(gray, edges, 50, 150);

        // Find contours in the edges
        vector<vector<Point>> contours;
        findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Create a copy of the input image to draw on
        Mat outputImage = inputImage.clone();

        // Loop through the contours and draw them
        for (size_t i = 0; i < contours.size(); i++) {
            Scalar color = Scalar(0, 255, 0); // Green color for the outlines
            drawContours(outputImage, contours, static_cast<int>(i), color, 2);
        }

        // Display the original image with overlays
        imshow("Shape Detection", outputImage);
        waitKey(0); // Wait for a key press to close the window
    }

private:
    string imagePath; // Path to the image
};

int main() {
    // Hard-coded image path
    const string imagePath = "apple.jpg"; // Replace with your actual image filename

    // Create a ShapeDetect object
    ShapeDetect shapeDetector(imagePath);

    // Execute shape detection
    shapeDetector.detectAndDrawShapes();

    return 0;
}
