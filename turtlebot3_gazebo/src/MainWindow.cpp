#include "MainWindow.hpp"
#include <QPixmap>
#include <QImage>
#include <QPainter>

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent)
{
  // Initialize ROS2 node
  node_ = rclcpp::Node::make_shared("inventory_gui_node");

  // Initialize tf2_ros buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize InventoryWaypointsNode with the shared node
  inventory_node_ = std::make_shared<InventoryWaypointsNode>(node_);

  setupUi();
  startRosSpin();

  // Subscribe to map and marker topics
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_ = msg;
      RCLCPP_INFO(node_->get_logger(), "Received map data");
    });

  marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
    "waypoint_markers", 10,
    [this](visualization_msgs::msg::Marker::SharedPtr msg) {
      waypoints_.push_back(*msg);
      RCLCPP_INFO(node_->get_logger(), "Received waypoint marker");
    });

  // Update GUI periodically
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateMap);
  update_timer_->start(500); // Update every 500 ms

  // Connect start and stop buttons
  connect(start_button_, &QPushButton::clicked, [this]() {
    inventory_node_->start_navigation();
  });

  connect(stop_button_, &QPushButton::clicked, [this]() {
    inventory_node_->stop_navigation();
  });
}

void MainWindow::updateMap()
{
  if (!map_) {
    RCLCPP_INFO(node_->get_logger(), "Map data not available yet");
    return;
  }

  // Get robot pose via TF
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    // Adjust frame names as necessary
    transformStamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not transform 'map' to 'base_link': %s", ex.what());
    return;
  }

  // Proceed to draw the map and robot position
  int width = map_->info.width;
  int height = map_->info.height;
  QImage image(width, height, QImage::Format_RGB888);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = x + (height - y - 1) * width;
      int value = map_->data[index];
      QColor color;

      if (value == -1) {
        color = Qt::gray; // Unknown
      } else if (value == 0) {
        color = Qt::white; // Free space
      } else {
        color = Qt::black; // Occupied
      }

      image.setPixelColor(x, y, color);
    }
  }

  // Draw robot position and waypoints
  QPainter painter(&image);

  // Draw waypoints
  painter.setBrush(Qt::red);
  for (const auto &marker : waypoints_) {
    int x = (marker.pose.position.x - map_->info.origin.position.x) / map_->info.resolution;
    int y = height - (marker.pose.position.y - map_->info.origin.position.y) / map_->info.resolution;
    painter.drawEllipse(QPointF(x, y), 3, 3);
  }

  // Draw robot position
  painter.setBrush(Qt::blue);
  double robot_x_m = transformStamped.transform.translation.x;
  double robot_y_m = transformStamped.transform.translation.y;
  int robot_x = (robot_x_m - map_->info.origin.position.x) / map_->info.resolution;
  int robot_y = height - (robot_y_m - map_->info.origin.position.y) / map_->info.resolution;
  painter.drawEllipse(QPointF(robot_x, robot_y), 5, 5);

  painter.end();

  // Display the image
  map_label_->setPixmap(QPixmap::fromImage(image).scaled(map_label_->size(), Qt::KeepAspectRatio));
}
