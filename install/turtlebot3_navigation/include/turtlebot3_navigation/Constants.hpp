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
  constexpr double CENTER_LIDAR_ANGLE = 0;                            // Angle for center LIDAR scan (forward-facing)
  constexpr double LEFT_LIDAR_ANGLE = 60;                             // Angle for left LIDAR scan (60 degrees left)
  constexpr double RIGHT_LIDAR_ANGLE = 300;                           // Angle for right LIDAR scan (60 degrees right, measured clockwise)
  constexpr int NUM_SCAN_POSITIONS = 3;                               // Number of specific scan angles to be used
  constexpr double SCAN_ANGLE[3] = {CENTER_LIDAR_ANGLE, LEFT_LIDAR_ANGLE, RIGHT_LIDAR_ANGLE};  // Array storing scan angles for different directions
  constexpr double MINIMUM_LEFT_WALL_DISTANCE = 0.3;                  // Minimum acceptable distance from the left wall
  constexpr double MAXIMUM_LEFT_WALL_DISTANCE = 0.5;                  // Maximum acceptable distance from the left wall
  constexpr double MINIMUM_FRONT_WALL_DISTANCE = 0.9;                 // Minimum acceptable distance from the front wall to avoid collisions
  constexpr double IDEAL_LEFT_WALL_DISTANCE = 0.6;                    // Target distance from the left wall for smooth navigation

  // --- Movement Parameters ---
  constexpr double LINEAR_VELOCITY = 0.4;                            // Default forward speed for the robot
  constexpr double OBSTACLE_AVOIDANCE_ANGULAR_VELOCITY = 0.7;       // Speed of angular rotation for obstacle avoidance
  constexpr double PROPORTIONAL_TURNING_FACTOR = 0.4;                // Proportional gain for turning adjustment (P-controller)
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
