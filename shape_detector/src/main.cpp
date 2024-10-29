#include <rclcpp/rclcpp.hpp>
#include "shape_detector.hpp"


int main(int argc, char **argv)
{
    // Initialise ROS2 with command line
    rclcpp::init(argc, argv);

    // Check for command line arguments to clear the output file
    bool clear_file = false;
    for (int i = 1; i < argc; ++i)
    {
        // Check if the clear command is added in the command
        if (std::string(argv[i]) == "--clear")
        {
            clear_file = true;
            break;
        }
    }

    // Clear the output file if the flag is provided
    if (clear_file)
    {
        // Open the output of the file and clear the file. 
        std::ofstream output_file("detected_shapes.txt", std::ios::out | std::ios::trunc);
        output_file.close();
        std::cout << "Output file cleared." << std::endl;
        
        // Terminate the program after clearing the file
        rclcpp::shutdown();
        return 0; // Exit the program
    }

    auto node = std::make_shared<ShapeDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

