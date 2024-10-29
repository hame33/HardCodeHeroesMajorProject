// MainWindow.cpp

// --- Includes ---
#include "MainWindow.hpp"
#include <QPixmap>
#include <QImage>
#include <QPainter>

// --- MainWindow Implementation ---

// --- Constructor ---
MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent), should_display_map_(true)
{
  // Initialize ROS2 node
  node_ = rclcpp::Node::make_shared("inventory_gui_node");

  // Initialize TF2 components for coordinate transformations
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize InventoryWaypointsNode with ROS2 node
  inventory_node_ = std::make_shared<InventoryWaypointsNode>(node_);

  initializeUi();
  startRosSpin();

  // Initialize subscribers
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(10).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = msg; });

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robot_pose", 10,
    [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      robot_pose_ = msg;
      RCLCPP_INFO(node_->get_logger(), "Received robot pose");
    });

  marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
    "waypoint_markers", 10,
    [this](visualization_msgs::msg::Marker::SharedPtr msg) {
      waypoints_.push_back(*msg);
    });

  // Initialize GUI refresh timer
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &MainWindow::updateMap);
  update_timer_->start(500);

  // Connect buttons to actions
  connect(start_button_, &QPushButton::clicked, [this]() {
    inventory_node_->start_navigation();
    should_display_map_ = true;
  });

  connect(stop_button_, &QPushButton::clicked, [this]() {
    inventory_node_->stop_navigation();
  });

  connect(reset_button_, &QPushButton::clicked, this, &MainWindow::resetMap);
}

// --- Destructor ---
MainWindow::~MainWindow()
{
  rclcpp::shutdown();
  if (ros_spin_thread_ && ros_spin_thread_->joinable()) 
  {
    ros_spin_thread_->join();
  }
}

// --- initializeUi ---
void MainWindow::initializeUi()
{
  // Initialize GUI components
  map_label_ = new QLabel("Map will be displayed here.");
  map_label_->setAlignment(Qt::AlignCenter);

  start_button_ = new QPushButton("Start Navigation");
  stop_button_ = new QPushButton("Stop Navigation");
  reset_button_ = new QPushButton("Reset Map");

  QHBoxLayout *button_layout = new QHBoxLayout();
  button_layout->addWidget(start_button_);
  button_layout->addWidget(stop_button_);
  button_layout->addWidget(reset_button_);

  main_layout_ = new QVBoxLayout();
  main_layout_->addWidget(map_label_);
  main_layout_->addLayout(button_layout);

  central_widget_ = new QWidget(this);
  central_widget_->setLayout(main_layout_);
  setCentralWidget(central_widget_);

  setWindowTitle("Inventory GUI");
  resize(800, 600);
}

// --- startRosSpin ---
void MainWindow::startRosSpin()
{
  ros_spin_thread_ = std::make_shared<std::thread>([this]() {
    rclcpp::spin(node_);
  });
}

// --- updateMap ---
void MainWindow::updateMap()
{
  if (!should_display_map_ || !map_) 
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for map data...");
    return;
  }

  // Generate map image based on occupancy grid data
  int width = map_->info.width;
  int height = map_->info.height;
  QImage image(width, height, QImage::Format_RGB888);

  // Map occupancy grid values to colors
  for (int y = 0; y < height; ++y) 
  {
    for (int x = 0; x < width; ++x) 
    {
      int index = x + (height - y - 1) * width;
      int8_t value = map_->data[index];
      QColor color = (value == -1) ? Qt::gray : (value == 0 ? Qt::white : Qt::black);
      image.setPixelColor(x, y, color);
    }
  }

  // Draw robot position if available
  if (robot_pose_) 
  {
    QPainter painter(&image);
    painter.setPen(QPen(Qt::blue, 2));
    painter.setBrush(Qt::blue);

    double robot_x = robot_pose_->pose.position.x;
    double robot_y = robot_pose_->pose.position.y;
    int x = (robot_x - map_->info.origin.position.x) / map_->info.resolution;
    int y = height - ((robot_y - map_->info.origin.position.y) / map_->info.resolution);

    if (x >= 0 && x < width && y >= 0 && y < height) 
    {
      painter.drawEllipse(QPointF(x, y), 5, 5);
    }
    painter.end();
  }

  // Draw frontiers if available
  if (frontiers_) 
  {
    QPainter painter(&image);
    painter.setPen(QPen(Qt::red, 1));
    for (const auto &point : frontiers_->cells) 
    {
      int x = (point.x - map_->info.origin.position.x) / map_->info.resolution;
      int y = height - ((point.y - map_->info.origin.position.y) / map_->info.resolution);
      if (x >= 0 && x < width && y >= 0 && y < height) 
      {
        painter.drawPoint(x, y);
      }
    }
    painter.end();
  }

  // Display updated map image in the QLabel
  map_label_->setPixmap(QPixmap::fromImage(image).scaled(map_label_->size(), Qt::KeepAspectRatio));
}

// --- resetMap ---
void MainWindow::resetMap()
{
  map_.reset();
  frontiers_.reset();
  map_label_->clear();
  map_label_->setText("Map has been reset.");
  inventory_node_->stop_navigation();
  RCLCPP_INFO(node_->get_logger(), "Map has been reset.");
}
