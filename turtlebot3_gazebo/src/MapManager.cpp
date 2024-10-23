#include "MapManager.hpp"
#include "WaypointManager.hpp"
#include <iostream>
#include <utility>
#include <vector>

// --- MapManager Class Implementation ---

// --- Constructor ---
MapManager::MapManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener, std::shared_ptr<SensorProcessor> sensor_processor)
: tf_buffer_(tf_buffer), tf_listener_(tf_listener), sensor_processor_(sensor_processor)
{
  robot_grid_x_pos_ = 0;
  robot_grid_y_pos_ = 0;
  closest_frontier_ = std::make_pair(0.0,0.0);
  closest_frontier_pixels_ = std::make_pair(0,0);
}

MapManager::MapManager(std::shared_ptr<SensorProcessor> sensor_processor, std::shared_ptr<WaypointManager> waypoint_manager)
: sensor_processor_(sensor_processor), waypoint_manager_(waypoint_manager)
{
  robot_grid_x_pos_ = 0;
  robot_grid_y_pos_ = 0;
  closest_frontier_ = std::make_pair(0.0,0.0);
  closest_frontier_pixels_ = std::make_pair(0,0);
}

// --- process_map_data ---
void MapManager::process_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg) 
{    
  std::cout << "map_callback" << std::endl;
  std::cout << "Received map with size: " << static_cast<int>(ocp_grid_msg->info.width) << " by " << static_cast<int>(ocp_grid_msg->info.height) << std::endl;

  int min_cell_value = find_min_cell_value(ocp_grid_msg);

  for (int row = static_cast<int>(ocp_grid_msg->info.height) - 1; row > 0; row--) 
  {
    for (int col = 0; col < static_cast<int>(ocp_grid_msg->info.width); col++)
    {
      int index = static_cast<int>(ocp_grid_msg->info.width) * row + col;
      int cell_value = ocp_grid_msg->data[index]; 
      if (min_cell_value > Constants::MAX_FREE_SPACE_VALUE)
      {
        if (cell_value <= -1)
        {
          std::cout << "?";
        }
        else if (cell_value > min_cell_value)
        {
          std::cout << "|";
        }
        else
        {
          std::cout << " ";
          search_for_frontiers(ocp_grid_msg, col, row);
        }
      }
      else 
      {
        if (cell_value <= -1)
        {
          std::cout << "?";
        }
        else if (cell_value > Constants::MAX_FREE_SPACE_VALUE)
        {
          std::cout << "|";
        }
        else
        {
          std::cout << " ";
          search_for_frontiers(ocp_grid_msg, col, row);
        }
      }
    }
    std::cout << std::endl;
  }

  double robot_x_pos = sensor_processor_->get_robot_x_pos();
  double robot_y_pos = sensor_processor_->get_robot_y_pos();

  robot_grid_x_pos_ = std::abs(ocp_grid_msg->info.origin.position.x) / ocp_grid_msg->info.resolution;
  robot_grid_y_pos_ = std::abs(ocp_grid_msg->info.origin.position.y) / ocp_grid_msg->info.resolution;

  find_closest_frontier(ocp_grid_msg);
  // print_frontier_pixels();
  auto waypoint_manager = waypoint_manager_.lock();
  waypoint_manager->print_completed_goals();
  
  std::cout << "Grid map origin: " << ocp_grid_msg->info.origin.position.x << " " << ocp_grid_msg->info.origin.position.y << std::endl;
  std::cout << "Robot world coordinates: (" << robot_x_pos << "," << robot_y_pos << ")" << std::endl;
  std::cout << "Robot grid coordinates: (" << robot_grid_x_pos_ << "," << robot_grid_y_pos_ << ")" << std::endl;
  std::cout << "Closest Frontier: (" << closest_frontier_.first << "," << closest_frontier_.second << ")" << std::endl;

  frontier_pixels_.clear();
}

// --- search_for_frontiers ---
void MapManager::search_for_frontiers(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg, int grid_cell_x, int grid_cell_y)
{
  int index = static_cast<int>(ocp_grid_msg->info.width) * grid_cell_y + grid_cell_x;
  if (ocp_grid_msg->data[index - static_cast<int>(ocp_grid_msg->info.width)] <= -1)
  {
    std::pair<int, int> frontier_pixel = std::make_pair(grid_cell_x, grid_cell_y);
    frontier_pixels_.push_back(frontier_pixel);
  }
  else if (ocp_grid_msg->data[index + static_cast<int>(ocp_grid_msg->info.width)] <= -1)
  {
    std::pair<int, int> frontier_pixel = std::make_pair(grid_cell_x, grid_cell_y);
    frontier_pixels_.push_back(frontier_pixel);
  }
  else if (ocp_grid_msg->data[index - 1] <= -1)
  {
    std::pair<int, int> frontier_pixel = std::make_pair(grid_cell_x, grid_cell_y);
    frontier_pixels_.push_back(frontier_pixel);
  }
  else if (ocp_grid_msg->data[index + 1] <= -1)
  {
    std::pair<int, int> frontier_pixel = std::make_pair(grid_cell_x, grid_cell_y);
    frontier_pixels_.push_back(frontier_pixel);
  }
}

// --- print_frontier_pixels ---
void MapManager::print_frontier_pixels() 
{
  std::cout << "Frontier Pixels:" << std::endl;
  for (const auto& pixel : frontier_pixels_) {
      std::cout << "(" << pixel.first << ", " << pixel.second << ")" << std::endl;
  }
}

// --- find_min_cell_value ---
int MapManager::find_min_cell_value(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  int min_cell_value = 100;
  for (int i = 0; i < static_cast<int>(ocp_grid_msg->info.width * ocp_grid_msg->info.height); i++)
  {
    if ((ocp_grid_msg->data[i] < min_cell_value) && (ocp_grid_msg->data[i] > -1))
    {
      min_cell_value = ocp_grid_msg->data[i];
    }
  }

  return min_cell_value;
}

// --- find_closest_frontier ---
void MapManager::find_closest_frontier(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  double shortest_distance = 0.0;

  for (auto it = frontier_pixels_.begin(); it != frontier_pixels_.end(); )
{
  const auto& pixel = *it;

  int current_x_distance = static_cast<int>(robot_grid_x_pos_) - static_cast<int>(pixel.first);
  int current_y_distance = static_cast<int>(robot_grid_y_pos_) - static_cast<int>(pixel.second);
  double current_distance = std::sqrt((current_x_distance * current_x_distance) + (current_y_distance * current_y_distance));

  if (shortest_distance == 0.0)
  {
    shortest_distance = current_distance;
  }
  else if (current_distance < shortest_distance)
  {
    shortest_distance = current_distance;
    closest_frontier_pixels_.first = pixel.first;
    closest_frontier_pixels_.second = pixel.second;

    check_walls_at_frontier(ocp_grid_msg, closest_frontier_pixels_.first, closest_frontier_pixels_.second);
    closest_frontier_.first = (closest_frontier_pixels_.first - robot_grid_x_pos_) * ocp_grid_msg->info.resolution;
    closest_frontier_.second = (closest_frontier_pixels_.second - robot_grid_y_pos_) * ocp_grid_msg->info.resolution;

    // Check if closest_frontier_ is a completed goal
    auto waypoint_manager = waypoint_manager_.lock();
    std::vector<std::pair<double, double>> completed_goals = waypoint_manager->get_completed_goals();
    double epsilon = Constants::COMPLETED_GOAL_ERROR; // Tolerance for floating-point comparison of completed goals

    auto completed_it = std::find_if(completed_goals.begin(), completed_goals.end(),
      [this, epsilon](const std::pair<double, double>& c_goal) 
      {
        return std::abs(c_goal.first - this->closest_frontier_.first) < epsilon &&
               std::abs(c_goal.second - this->closest_frontier_.second) < epsilon;
      });

    if (completed_it != completed_goals.end()) 
    {
      // Remove the current pixel from frontier_pixels_
      frontier_pixels_.erase(it);
      find_closest_frontier(ocp_grid_msg);
      break;
    } 
    else 
    {
      ++it;
    }
  }
  else
  {
    ++it;
  }
}

}

// --- check_walls_at_frontier ---
void MapManager::check_walls_at_frontier(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg, int& frontier_pixel_x, int& frontier_pixel_y)
{
  // Check if x is too close to wall
  for (int i = -1 * Constants::MIN_PIXELS_FROM_WALL;  i < Constants::MIN_PIXELS_FROM_WALL; i++)
  {
    int current_index = frontier_pixel_y * static_cast<int>(ocp_grid_msg->info.width) + frontier_pixel_x + i;
    if (ocp_grid_msg->data[current_index] > 80)
    {
      if (i < 0)
      {
        frontier_pixel_x = frontier_pixel_x + Constants::MIN_PIXELS_FROM_WALL - std::abs(i);
        break;
      }
      else if (i > 0)
      {
        frontier_pixel_x = frontier_pixel_x - Constants::MIN_PIXELS_FROM_WALL - std::abs(i);
        break;
      }
    }
  }

  // Check if y is too close to wall
  for (int i = -1 * Constants::MIN_PIXELS_FROM_WALL;  i < Constants::MIN_PIXELS_FROM_WALL; i++)
  {
    int current_index = (frontier_pixel_y + i) * static_cast<int>(ocp_grid_msg->info.width) + frontier_pixel_x;
    if (ocp_grid_msg->data[current_index] > 80)
    {
      if (i < 0)
      {
        frontier_pixel_y = frontier_pixel_y - Constants::MIN_PIXELS_FROM_WALL - std::abs(i);
        break;
      }
      else if (i > 0)
      {
        frontier_pixel_x = frontier_pixel_y + Constants::MIN_PIXELS_FROM_WALL - std::abs(i);
        break;
      }
    }
  }
}

// --- get_closest_frontier ---
std::pair<double,double> MapManager::get_closest_frontier() const
{
  return closest_frontier_;
}
