#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

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

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private slots:
  void updateMap();

private:
  void setupUi();
  void startRosSpin();

  // ROS components
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<InventoryWaypointsNode> inventory_node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr frontier_sub_;
    nav_msgs::msg::GridCells::SharedPtr frontiers_;

  // GUI components
  QLabel *map_label_;
  QPushButton *start_button_;
  QPushButton *stop_button_;
  QVBoxLayout *main_layout_;
  QWidget *central_widget_;
  QTimer *update_timer_;

  // Data
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  geometry_msgs::msg::PoseStamped::SharedPtr robot_pose_;
  std::vector<visualization_msgs::msg::Marker> waypoints_;

  // ROS spin thread
  std::shared_ptr<std::thread> ros_spin_thread_;
};

#endif // MAIN_WINDOW_HPP