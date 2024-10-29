#include "MapManager.hpp"
#include "WaypointManager.hpp"
#include <iostream>
#include <utility>
#include <vector>

// --- MapManager Implementation ---

// --- Constructor ---
MapManager::MapManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener, std::shared_ptr<SensorProcessor> sensor_processor)
  : tf_buffer_(tf_buffer), tf_listener_(tf_listener), sensor_processor_(sensor_processor)
{
  robot_grid_x_pos_ = 0;
  robot_grid_y_pos_ = 0;
  closest_frontier_ = std::make_pair(0.0, 0.0);
  closest_frontier_pixels_ = std::make_pair(0, 0);
}

MapManager::MapManager(std::shared_ptr<SensorProcessor> sensor_processor, std::shared_ptr<WaypointManager> waypoint_manager)
  : sensor_processor_(sensor_processor), waypoint_manager_(waypoint_manager)
{
  robot_grid_x_pos_ = 0;
  robot_grid_y_pos_ = 0;
  closest_frontier_ = std::make_pair(0.0, 0.0);
  closest_frontier_pixels_ = std::make_pair(0, 0);
}

// --- process_map_data ---
void MapManager::process_map_data(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg) 
{    
  std::cout << "map_callback" << std::endl;
  std::cout << "Received map with size: " << static_cast<int>(ocp_grid_msg->info.width) 
            << " by " << static_cast<int>(ocp_grid_msg->info.height) << std::endl;

  int min_cell_value = find_min_cell_value(ocp_grid_msg);

  // Set robot grid coordinates based on map origin and resolution
  robot_grid_x_pos_ = std::abs(ocp_grid_msg->info.origin.position.x) / ocp_grid_msg->info.resolution;
  robot_grid_y_pos_ = std::abs(ocp_grid_msg->info.origin.position.y) / ocp_grid_msg->info.resolution;

  find_frontiers_with_bfs(ocp_grid_msg);
  find_closest_frontier(ocp_grid_msg);

  if (frontier_pixels_.empty())
  {
    auto waypoint_manager = waypoint_manager_.lock();
    waypoint_manager->publish_return_to_start();
    closest_frontier_ = {0.0, 0.0};
  }

  frontier_pixels_.clear();
}

// --- search_for_frontiers ---
void MapManager::search_for_frontiers(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg, int grid_cell_x, int grid_cell_y)
{
  int index = static_cast<int>(ocp_grid_msg->info.width) * grid_cell_y + grid_cell_x;

  // Check adjacent cells for frontiers and add them if found
  if (ocp_grid_msg->data[index - static_cast<int>(ocp_grid_msg->info.width)] <= -1 ||
      ocp_grid_msg->data[index + static_cast<int>(ocp_grid_msg->info.width)] <= -1 ||
      ocp_grid_msg->data[index - 1] <= -1 || ocp_grid_msg->data[index + 1] <= -1)
  {
    frontier_pixels_.emplace_back(grid_cell_x, grid_cell_y);
  }
}

// --- print_frontier_pixels ---
void MapManager::print_frontier_pixels() 
{
  std::cout << "Frontier Pixels:" << std::endl;
  for (const auto& pixel : frontier_pixels_)
  {
    std::cout << "(" << pixel.first << ", " << pixel.second << ")" << std::endl;
  }
}

// --- find_frontiers_with_bfs ---
void MapManager::find_frontiers_with_bfs(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  int width = ocp_grid_msg->info.width;
  int height = ocp_grid_msg->info.height;
  std::vector<bool> visited(width * height, false);
  std::queue<std::pair<int, int>> queue;

  // Robot position in grid coordinates
  int robot_x = static_cast<int>(std::abs((ocp_grid_msg->info.origin.position.x)) / ocp_grid_msg->info.resolution);
  int robot_y = static_cast<int>(std::abs((ocp_grid_msg->info.origin.position.y)) / ocp_grid_msg->info.resolution);

  std::cout << "Robot_x " << robot_x << " Robot_y " << robot_y << std::endl;
  if (robot_x < 0 || robot_x >= width || robot_y < 0 || robot_y >= height) 
  {
    std::cerr << "Error: Robot position is outside map bounds." << std::endl;
    return;
  }

  queue.emplace(robot_x, robot_y);

  while (!queue.empty())
  {
    auto [x, y] = queue.front();
    queue.pop();

    if (x < 0 || x >= width || y < 0 || y >= height || visited[y * width + x])
    {
      continue;
    }

    visited[y * width + x] = true;
    int cell_value = ocp_grid_msg->data[y * width + x];

    if (cell_value == -1 || cell_value > Constants::MAX_FREE_SPACE_VALUE) 
    {
      continue;
    }

    // Identify frontier pixels
    bool is_frontier = false;
    for (int dy = -1; dy <= 1; dy++)
    {
      for (int dx = -1; dx <= 1; dx++)
      {
        int nx = x + dx, ny = y + dy;
        if (nx >= 0 && nx < width && ny >= 0 && ny < height && ocp_grid_msg->data[ny * width + nx] == -1)
        {
          is_frontier = true;
          break;
        }
      }
      if (is_frontier) break;
    }

    if (is_frontier)
    {
      frontier_pixels_.emplace_back(x, y);
    } 
    else 
    {
      // Enqueue neighboring cells
      for (int dy = -1; dy <= 1; dy++)
      {
        for (int dx = -1; dx <= 1; dx++)
        {
          int nx = x + dx, ny = y + dy;
          if (nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny * width + nx])
          {
            queue.emplace(nx, ny);
          }
        }
      }
    }
  }
}

// --- find_min_cell_value ---
int MapManager::find_min_cell_value(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  int min_cell_value = Constants::MAX_CELL_VALUE;
  for (int i = 0; i < static_cast<int>(ocp_grid_msg->info.width * ocp_grid_msg->info.height); i++)
  {
    if (ocp_grid_msg->data[i] > -1 && ocp_grid_msg->data[i] < min_cell_value)
    {
      min_cell_value = ocp_grid_msg->data[i];
    }
  }
  return min_cell_value;
}

// --- find_closest_frontier ---
void MapManager::find_closest_frontier(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg)
{
  double shortest_distance = std::numeric_limits<double>::max();

  for (auto it = frontier_pixels_.begin(); it != frontier_pixels_.end();)
  {
    const auto& pixel = *it;
    double current_distance = std::hypot(robot_grid_x_pos_ - pixel.first, robot_grid_y_pos_ - pixel.second);

    if (current_distance < shortest_distance)
    {
      shortest_distance = current_distance;
      closest_frontier_pixels_ = pixel;
      check_walls_at_frontier(ocp_grid_msg, closest_frontier_pixels_.first, closest_frontier_pixels_.second);

      closest_frontier_ = {(closest_frontier_pixels_.first - robot_grid_x_pos_) * ocp_grid_msg->info.resolution,
                           (closest_frontier_pixels_.second - robot_grid_y_pos_) * ocp_grid_msg->info.resolution};

      auto waypoint_manager = waypoint_manager_.lock();
      auto completed_goals = waypoint_manager->get_completed_goals();
      if (std::any_of(completed_goals.begin(), completed_goals.end(),
                      [this](const auto& goal) { return std::abs(goal.first - closest_frontier_.first) < Constants::COMPLETED_GOAL_ERROR &&
                                                      std::abs(goal.second - closest_frontier_.second) < Constants::COMPLETED_GOAL_ERROR; }))
      {
        it = frontier_pixels_.erase(it);
        find_closest_frontier(ocp_grid_msg);
        return;
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
  print_frontier_pixels();
}

// --- check_walls_at_frontier ---
void MapManager::check_walls_at_frontier(const nav_msgs::msg::OccupancyGrid::SharedPtr ocp_grid_msg, int& frontier_pixel_x, int& frontier_pixel_y)
{
  int adjust_x = 0, adjust_y = 0;

  for (int dy = -Constants::MIN_PIXELS_FROM_WALL; dy <= Constants::MIN_PIXELS_FROM_WALL; dy++)
  {
    for (int dx = -Constants::MIN_PIXELS_FROM_WALL; dx <= Constants::MIN_PIXELS_FROM_WALL; dx++)
    {
      int current_x = frontier_pixel_x + dx;
      int current_y = frontier_pixel_y + dy;

      if (current_x < 0 || current_x >= ocp_grid_msg->info.width || current_y < 0 || current_y >= ocp_grid_msg->info.height)
      {
        continue;
      }

      int current_index = current_y * ocp_grid_msg->info.width + current_x;
      if (ocp_grid_msg->data[current_index] > Constants::WALL_CHECK_VALUE)
      {
        adjust_x += dx < 0 ? Constants::MIN_PIXELS_FROM_WALL - std::abs(dx) : -Constants::MIN_PIXELS_FROM_WALL + std::abs(dx);
        adjust_y += dy < 0 ? Constants::MIN_PIXELS_FROM_WALL - std::abs(dy) : -Constants::MIN_PIXELS_FROM_WALL + std::abs(dy);

        frontier_pixel_x = std::clamp(frontier_pixel_x + adjust_x, 0, ocp_grid_msg->info.width - 1);
        frontier_pixel_y = std::clamp(frontier_pixel_y + adjust_y, 0, ocp_grid_msg->info.height - 1);
      }
    }
  }
  std::cout << "Adjusted frontier pixel to (" << frontier_pixel_x << ", " << frontier_pixel_y << ") to maintain distance from walls." << std::endl;
}

// --- get_closest_frontier ---
std::pair<double, double> MapManager::get_closest_frontier() const
{
  return closest_frontier_;
}
