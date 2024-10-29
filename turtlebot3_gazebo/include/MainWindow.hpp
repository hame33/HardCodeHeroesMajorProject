#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

// --- Includes ---
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/grid_cells.hpp>

#include "InventoryWaypointsNode.hpp"

// --- MainWindow Class Interface ---
// MainWindow handles the main user interface for interacting with the robot’s 
// navigation and waypoint management system. It uses ROS to subscribe to topics 
// for map updates, robot pose, and markers. The class also manages GUI components 
// like start, stop, and reset buttons, and it regularly updates the displayed map 
// using a timer.
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
    // --- Constructor ---
    explicit MainWindow(QWidget *parent = nullptr);

    // --- Destructor ---
    ~MainWindow();

private slots:
    // updateMap - Updates the displayed map in the GUI
    void updateMap();

    // resetMap - Resets the map display in the GUI
    void resetMap();

private:
    // --- MainWindow Functions ---

    // setupUi - Initializes and configures the user interface elements
    void initializeUi();

    // startRosSpin - Starts the ROS spinning thread for message handling
    void startRosSpin();

    // --- ROS Core Node ---
    rclcpp::Node::SharedPtr node_;                                                // ROS node for communication
    std::shared_ptr<InventoryWaypointsNode> inventory_node_;                      // Manages inventory waypoints

    // --- ROS Subscriptions ---
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;       // Subscriber for map data
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;   // Subscriber for robot pose
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_; // Subscriber for waypoints markers
    rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr frontier_sub_;      // Subscriber for frontier cells

    // --- ROS TF2 Components ---
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                  // TF2 buffer for transformations
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                     // TF2 listener for transformations

    // --- GUI Components ---
    QLabel *map_label_;             // Label for displaying the map
    QPushButton *start_button_;     // Start button for initiating tasks
    QPushButton *stop_button_;      // Stop button for halting tasks
    QPushButton *reset_button_;     // Reset button to clear display
    QVBoxLayout *main_layout_;      // Main layout for central widget
    QWidget *central_widget_;       // Central widget in the main window
    QTimer *update_timer_;          // Timer to update the map display periodically

    // --- Data ---
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;                      // Stores the latest map data
    geometry_msgs::msg::PoseStamped::SharedPtr robot_pose_;            // Current pose of the robot
    std::vector<visualization_msgs::msg::Marker> waypoints_;           // Collection of waypoints for navigation
    nav_msgs::msg::GridCells::SharedPtr frontiers_;                    // Data for detected frontiers
    bool should_display_map_;                                          // Flag to control map display

    // --- ROS Spin Thread ---
    std::shared_ptr<std::thread> ros_spin_thread_;                     // Thread for handling ROS spin
};

#endif // MAIN_WINDOW_HPP
