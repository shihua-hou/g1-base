#include "g1_base_perception/nav_obstacle_cloud_filter.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_base_perception::NavObstacleCloudFilter>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(node->get_logger(), "nav_obstacle_cloud_filter failed: %s", exc.what());
  }
  rclcpp::shutdown();
  return 0;
}
