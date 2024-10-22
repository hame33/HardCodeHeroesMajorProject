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

  // Initialize InventoryWaypointsNode
  inventory_node_ = std::make_shared<InventoryWaypointsNode>(node_);

  setupUi();
  startRosSpin();

  // Subscribe to map, pose, and marker topics
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_ = msg;
    });

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robot_pose", 10,
    [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      robot_pose_ = msg;
    });

  marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
    "waypoint_markers", 10,
    [this](visualization_msgs::msg::Marker::SharedPtr msg) {
      waypoints_.push_back(*msg);
    });

  // Update GUI periodically
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateMap);
  update_timer_->start(500); // Update every 500 ms
}

MainWindow::~MainWindow()
{
  rclcpp::shutdown();
  if (ros_spin_thread_ && ros_spin_thread_->joinable()) {
    ros_spin_thread_->join();
  }
}

void MainWindow::setupUi()
{
  map_label_ = new QLabel("Map will be displayed here.");
  map_label_->setAlignment(Qt::AlignCenter);

  start_button_ = new QPushButton("Start Navigation");
  stop_button_ = new QPushButton("Stop Navigation");

  QHBoxLayout *button_layout = new QHBoxLayout();
  button_layout->addWidget(start_button_);
  button_layout->addWidget(stop_button_);

  main_layout_ = new QVBoxLayout();
  main_layout_->addWidget(map_label_);
  main_layout_->addLayout(button_layout);

  central_widget_ = new QWidget(this);
  central_widget_->setLayout(main_layout_);
  setCentralWidget(central_widget_);

  setWindowTitle("Inventory GUI");
  resize(800, 600);

  // Connect button signals to slots
  connect(start_button_, &QPushButton::clicked, [this]() {
    // Start navigation logic
    inventory_node_->start_navigation();
  });

  connect(stop_button_, &QPushButton::clicked, [this]() {
    // Stop navigation logic
    inventory_node_->stop_navigation();
  });
}

void MainWindow::startRosSpin()
{
  ros_spin_thread_ = std::make_shared<std::thread>([this]() {
    rclcpp::spin(node_);
  });
}

void MainWindow::updateMap()
{
  if (!map_ || !robot_pose_) {
    return;
  }

  // Convert the occupancy grid to an image
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
  int robot_x = (robot_pose_->pose.position.x - map_->info.origin.position.x) / map_->info.resolution;
  int robot_y = height - (robot_pose_->pose.position.y - map_->info.origin.position.y) / map_->info.resolution;
  painter.drawEllipse(QPointF(robot_x, robot_y), 5, 5);

  painter.end();

  // Display the image
  map_label_->setPixmap(QPixmap::fromImage(image).scaled(map_label_->size(), Qt::KeepAspectRatio));
}
