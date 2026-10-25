#include <chrono>
#include <cmath>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap")
    , costmap_(robot::CostmapCore(this->get_logger())) {
  using std::placeholders::_1;

  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 5, std::bind(&CostmapNode::lidar_callback, this, _1));
  publish_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidar_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  RCLCPP_INFO(this->get_logger(), "Received a scan: %zu scan numbers",
              scan->ranges.size());

  // Step 1: Initialize costmap
  initializeCostmap();

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      // Calculate grid coordinates
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: Inflate obstacles
  inflateObstacles();

  // Step 4: Publish costmap
  publishCostmap(scan);
}

void CostmapNode::initializeCostmap() {}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid,
                                int &y_grid) {
  x_grid = std::round(range * std::cos(angle) * CostmapNode::RESOLUTION * 100 +
                      std::floor(SIZE / 2));
  y_grid = std::round(range * std::sin(angle) * CostmapNode::RESOLUTION * 100 +
                      std::floor(SIZE / 2));
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  this->occupancyGrid[std::size(occupancyGrid) - y_grid][x_grid] = 100;
}

void CostmapNode::inflateObstacles() {}

void CostmapNode::publishCostmap(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // auto message = std_msgs::msg::String();
  // message.data = "Hello, ROS 2!";
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  // string_pub_->publish(message);
  auto occGridMsg = nav_msgs::msg::OccupancyGrid();
  occGridMsg.header = scan->header;

  auto info = nav_msgs::msg::MapMetaData();
  info.resolution = CostmapNode::RESOLUTION;
  info.width = CostmapNode::GRIDSIZE;
  info.height = CostmapNode::GRIDSIZE;
  occGridMsg.info = info;

  publish_->publish(occGridMsg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}