// main.cpp

#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "MainWindow.hpp"

int main(int argc, char *argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Initialize Qt application
  QApplication app(argc, argv);

  // Create an instance of your main window
  MainWindow window;
  window.show();

  // Start the Qt event loop
  int result = app.exec();

  // Shutdown ROS2
  rclcpp::shutdown();

  return result;
}