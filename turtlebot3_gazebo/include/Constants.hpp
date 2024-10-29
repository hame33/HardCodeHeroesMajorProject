#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

// --- Includes ---
#include <cmath>

// --- Constants Namespace Interface ---
// The Constants namespace contains configuration values for the robot's movement, 
// sensor scan angles, and timing. It also defines enumerations for LIDAR scan 
// positions and goal statuses, improving readability and modularity.
namespace Constants {
    // --- Math Constants ---
    constexpr double DEG2RAD = M_PI / 180.0;    // Converts degrees to radians
    constexpr double RAD2DEG = 180.0 / M_PI;    // Converts radians to degrees

    // --- Scan Parameters ---
    constexpr int NUM_SCANNERS = 360;                                      // Number of lidar scanning points
    constexpr int NUM_SCAN_POSITIONS = 60;                                 // Number of specific scan angles to be used
    constexpr int ANGLE_BETWEEN_SCANS = NUM_SCANNERS / NUM_SCAN_POSITIONS; // Angle between scan positions
    constexpr double MAX_SCAN_DISTANCE = 3.5;                              // Maximum possible scan distance 

    // --- Map Parameters ---
    constexpr int MIN_PIXELS_FROM_WALL = 10;      // Minimum pixels from wall for safe navigation
    constexpr int MAX_FREE_SPACE_VALUE = 30;      // Maximum value for free space in occupancy grid
    constexpr int WALL_CHECK_VALUE = 10;          // Threshold for identifying walls in map data
    constexpr double COMPLETED_GOAL_ERROR = 1e-4; // Error tolerance for goal completion
    constexpr int MAX_CELL_VALUE = 100;           // Maximum cell value in occupancy grid

    enum PIXEL_ADJUSTMENT_STATUS {                // Enum for adjusting pixel position status
        NOT_NEEDED = 0,
        NEEDED = 1,
        COMPLETED = 2
    };

    // --- Goal Parameters ---
    enum GOAL_STATUS {                           // Enum for the different goal result options
        SUCCESS = 1,
        ABORTED = 2,
        CANCELED = 3
    };
}

#endif  // CONSTANTS_HPP_
