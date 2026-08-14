#ifndef G1_CENTERLINE_PLANNER__CENTERLINE_PLANNER_HPP_
#define G1_CENTERLINE_PLANNER__CENTERLINE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

namespace g1_centerline_planner
{

class CenterlinePlanner : public nav2_core::GlobalPlanner
{
public:
  CenterlinePlanner() = default;
  ~CenterlinePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct Cell
  {
    unsigned int x{0};
    unsigned int y{0};
  };

  struct SearchResult
  {
    bool success{false};
    std::vector<Cell> cells;
  };

  struct QueueEntry
  {
    unsigned int index{0};
    double f_score{0.0};
    double g_score{0.0};
  };

  unsigned int toIndex(unsigned int x, unsigned int y) const;
  Cell toCell(unsigned int index) const;
  bool isObstacleCost(unsigned char cost) const;
  bool isTraversable(unsigned int index, float required_clearance) const;
  double traversalCost(unsigned int from_index, unsigned int to_index, double step_m) const;
  double sequenceCost(const std::vector<Cell> & cells) const;
  double sequenceCost(const std::vector<Cell> & cells, std::size_t begin, std::size_t end) const;
  double heuristic(unsigned int index, unsigned int goal_index) const;

  std::vector<float> computeClearanceField() const;
  SearchResult search(
    unsigned int start_index,
    unsigned int goal_index,
    float required_clearance) const;
  std::vector<Cell> shortcutPath(const std::vector<Cell> & input) const;
  std::vector<Cell> densifyPath(const std::vector<Cell> & input) const;
  std::vector<Cell> cellsOnLine(const Cell & a, const Cell & b) const;
  bool hasLineOfSight(const Cell & a, const Cell & b, float required_clearance) const;
  bool findNearestTraversable(
    unsigned int seed_index,
    unsigned int * result_index,
    float required_clearance) const;

  geometry_msgs::msg::PoseStamped cellToPose(
    const Cell & cell,
    const std_msgs::msg::Header & header,
    double yaw) const;

  void assignPathOrientations(
    nav_msgs::msg::Path * path,
    const geometry_msgs::msg::PoseStamped & goal) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("CenterlinePlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  unsigned int width_{0};
  unsigned int height_{0};
  double resolution_{0.0};
  std::vector<float> clearance_m_;

  bool allow_unknown_{false};
  bool use_final_approach_orientation_{false};
  double min_clearance_{0.38};
  double target_clearance_{0.90};
  double shortcut_min_clearance_{0.45};
  double endpoint_snap_radius_{0.50};
  double endpoint_min_clearance_{0.20};
  double fallback_min_clearance_{0.25};
  double length_weight_{1.0};
  double wall_weight_{8.0};
  double center_weight_{4.0};
  double center_clearance_cap_{2.0};
  double costmap_weight_{2.0};
  double shortcut_cost_tolerance_{1.02};
};

}  // namespace g1_centerline_planner

#endif  // G1_CENTERLINE_PLANNER__CENTERLINE_PLANNER_HPP_
