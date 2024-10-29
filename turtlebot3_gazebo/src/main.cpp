// main.cpp

// --- Includes ---
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "MainWindow.hpp"

// --- Main Implementation ---
int main(int argc, char *argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Initialize Qt application
  QApplication app(argc, argv);

  // Create and show main window
  MainWindow window;
  window.show();

  // Start Qt event loop
  int result = app.exec();

  // Shutdown ROS2
  rclcpp::shutdown();

  return result;
}
