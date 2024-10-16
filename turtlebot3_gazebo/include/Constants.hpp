#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

// --- Includes ---

#include <cmath>

// --- Constants Namespace Interface ---
// The Constants namespace contains configuration values for the robot's movement, 
// sensor scan angles, and timing. 
// It also defines enumerations for LIDAR scan positions, improving readability.
namespace Constants {
  // --- Math Constants ---
  constexpr double DEG2RAD = M_PI / 180.0;    // Converts degrees to radians
  constexpr double RAD2DEG = 180.0 / M_PI;    // Converts radians to degrees

  // --- Scan Parameters ---
  constexpr int NUM_SCANNERS = 360;                                     // Number of lidar scanning points
  constexpr int NUM_SCAN_POSITIONS = 60;                               // Number of specific scan angles to be used
  constexpr int ANGLE_BETWEEN_SCANS = NUM_SCANNERS/NUM_SCAN_POSITIONS;   // Angle between scan positions
  constexpr double MAX_SCAN_DISTANCE = 3.5;                             // Maximum possible scan distance 

  // --- Movement Parameters ---
  constexpr double LINEAR_VELOCITY = 0.6;                            // Default forward speed for the robot
  constexpr double OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY = 0.75;       // Speed of angular rotation for obstacle avoidance
  constexpr double PROPORTIONAL_TURNING_FACTOR = 0.7;                // Proportional gain for turning adjustment (P-controller)
  constexpr double DERIVATIVE_TURNING_FACTOR = 0.07;                 // Derivative gain for turning adjustment (D-controller)
  constexpr double TURNING_ANGLE = 15 * DEG2RAD;                     // Default turning angle for the robot, converted to radians

  // --- Timer ---
  constexpr double DERIVATIVE_CONTROL_DELTA_T_FACTOR = 0.25;    // Delta T to divide the derivative error by

  // --- Enumerations ---
  enum class ScanPositions {    // Indexes for SCAN_ANGLE LIDAR readings 
    CENTER = 0,  
    LEFT,        
    RIGHT        
  };
}

#endif  // CONSTANTS_HPP_
