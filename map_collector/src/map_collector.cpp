#include <cstdio>
#include "map_collector/map_collector.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapCollector>());
  rclcpp::shutdown();
  return 0;
}
