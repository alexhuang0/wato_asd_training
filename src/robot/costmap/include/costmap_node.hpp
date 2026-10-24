#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
 
#include "costmap_core.hpp"

#include <array>
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    constexpr int RESOLUTION{0.1}; // in meters
    constexpr int SIZE{40} // in meters, determines size x size grid in m^2

    std::array<int, RESOLUTION * 100 * SIZE> OccupancyGrid{};
     
    // Place callback function here
    void publishMessage();
    void lidar_callback(const sensor_msgs::msg::LaserScan& sensorMsg);

  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};
 
#endif 