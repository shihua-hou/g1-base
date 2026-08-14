#include "g1_centerline_planner/centerline_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace g1_centerline_planner
{
namespace
{

constexpr float kInf = std::numeric_limits<float>::infinity();

double yawFromDelta(double dx, double dy)
{
  return std::atan2(dy, dx);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half = yaw * 0.5;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

}  // namespace

void CenterlinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("CenterlinePlanner failed to lock lifecycle node");
  }

  name_ = std::move(name);
  logger_ = node_->get_logger();
  clock_ = node_->get_clock();
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".min_clearance", rclcpp::ParameterValue(0.38));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".target_clearance", rclcpp::ParameterValue(0.90));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".shortcut_min_clearance", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".endpoint_snap_radius", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".endpoint_min_clearance", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".fallback_min_clearance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".length_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".wall_weight", rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".center_weight", rclcpp::ParameterValue(4.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".center_clearance_cap", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".costmap_weight", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".shortcut_cost_tolerance", rclcpp::ParameterValue(1.02));

  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node_->get_parameter(name_ + ".use_final_approach_orientation", use_final_approach_orientation_);
  node_->get_parameter(name_ + ".min_clearance", min_clearance_);
  node_->get_parameter(name_ + ".target_clearance", target_clearance_);
  node_->get_parameter(name_ + ".shortcut_min_clearance", shortcut_min_clearance_);
  node_->get_parameter(name_ + ".endpoint_snap_radius", endpoint_snap_radius_);
  node_->get_parameter(name_ + ".endpoint_min_clearance", endpoint_min_clearance_);
  node_->get_parameter(name_ + ".fallback_min_clearance", fallback_min_clearance_);
  node_->get_parameter(name_ + ".length_weight", length_weight_);
  node_->get_parameter(name_ + ".wall_weight", wall_weight_);
  node_->get_parameter(name_ + ".center_weight", center_weight_);
  node_->get_parameter(name_ + ".center_clearance_cap", center_clearance_cap_);
  node_->get_parameter(name_ + ".costmap_weight", costmap_weight_);
  node_->get_parameter(name_ + ".shortcut_cost_tolerance", shortcut_cost_tolerance_);

  if (target_clearance_ < min_clearance_) {
    RCLCPP_WARN(
      logger_,
      "target_clearance %.3f is below min_clearance %.3f; clamping target to min",
      target_clearance_, min_clearance_);
    target_clearance_ = min_clearance_;
  }
  shortcut_min_clearance_ = std::max(shortcut_min_clearance_, min_clearance_);
  endpoint_min_clearance_ = std::clamp(endpoint_min_clearance_, 0.0, min_clearance_);
  fallback_min_clearance_ = std::clamp(fallback_min_clearance_, 0.0, min_clearance_);

  center_clearance_cap_ = std::max(center_clearance_cap_, target_clearance_);
  shortcut_cost_tolerance_ = std::max(1.0, shortcut_cost_tolerance_);

  RCLCPP_INFO(
    logger_,
    "Configured %s: min_clearance=%.2f target_clearance=%.2f center_cap=%.2f "
    "wall_weight=%.2f center_weight=%.2f costmap_weight=%.2f allow_unknown=%s "
    "endpoint_min=%.2f fallback_min=%.2f",
    name_.c_str(), min_clearance_, target_clearance_, center_clearance_cap_,
    wall_weight_, center_weight_, costmap_weight_,
    allow_unknown_ ? "true" : "false", endpoint_min_clearance_, fallback_min_clearance_);
}

void CenterlinePlanner::cleanup()
{
  clearance_m_.clear();
  costmap_ = nullptr;
}

void CenterlinePlanner::activate() {}

void CenterlinePlanner::deactivate() {}

nav_msgs::msg::Path CenterlinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (costmap_ == nullptr) {
    throw std::runtime_error("CenterlinePlanner has no costmap");
  }
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw std::runtime_error(
            "CenterlinePlanner expects start and goal in frame '" + global_frame_ + "'");
  }

  width_ = costmap_->getSizeInCellsX();
  height_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  clearance_m_ = computeClearanceField();

  unsigned int start_x = 0;
  unsigned int start_y = 0;
  unsigned int goal_x = 0;
  unsigned int goal_y = 0;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    throw std::runtime_error("CenterlinePlanner start is outside the costmap");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    throw std::runtime_error("CenterlinePlanner goal is outside the costmap");
  }

  const unsigned int original_start_index = toIndex(start_x, start_y);
  const unsigned int original_goal_index = toIndex(goal_x, goal_y);
  unsigned int start_index = original_start_index;
  unsigned int goal_index = original_goal_index;
  const auto strict_clearance = static_cast<float>(min_clearance_);
  const auto endpoint_clearance = static_cast<float>(endpoint_min_clearance_);
  const auto fallback_clearance = static_cast<float>(fallback_min_clearance_);

  const bool found_strict_start =
    findNearestTraversable(original_start_index, &start_index, strict_clearance);
  const bool found_strict_goal =
    findNearestTraversable(original_goal_index, &goal_index, strict_clearance);

  if (!found_strict_start || !found_strict_goal) {
    unsigned int relaxed_start_index = start_index;
    unsigned int relaxed_goal_index = goal_index;
    if (
      !found_strict_start &&
      !findNearestTraversable(original_start_index, &relaxed_start_index, endpoint_clearance))
    {
      throw std::runtime_error("CenterlinePlanner could not find a valid start cell");
    }
    if (
      !found_strict_goal &&
      !findNearestTraversable(original_goal_index, &relaxed_goal_index, endpoint_clearance))
    {
      throw std::runtime_error("CenterlinePlanner could not find a valid goal cell");
    }
    RCLCPP_WARN(
      logger_,
      "CenterlinePlanner relaxed endpoint clearance from %.2f m to %.2f m",
      min_clearance_, endpoint_min_clearance_);
    start_index = relaxed_start_index;
    goal_index = relaxed_goal_index;
  }

  auto result = search(start_index, goal_index, strict_clearance);
  if (!result.success && fallback_clearance < strict_clearance) {
    unsigned int fallback_start_index = original_start_index;
    unsigned int fallback_goal_index = original_goal_index;
    const bool found_fallback_start =
      findNearestTraversable(original_start_index, &fallback_start_index, fallback_clearance);
    const bool found_fallback_goal =
      findNearestTraversable(original_goal_index, &fallback_goal_index, fallback_clearance);
    if (found_fallback_start && found_fallback_goal) {
      RCLCPP_WARN(
        logger_,
        "CenterlinePlanner strict search failed at %.2f m; retrying with %.2f m clearance",
        min_clearance_, fallback_min_clearance_);
      start_index = fallback_start_index;
      goal_index = fallback_goal_index;
      result = search(start_index, goal_index, fallback_clearance);
    }
  }
  if (!result.success) {
    throw std::runtime_error("CenterlinePlanner failed to find a path");
  }

  const auto shortened = shortcutPath(result.cells);
  const auto dense = densifyPath(shortened);

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  path.poses.reserve(dense.size());
  for (const auto & cell : dense) {
    path.poses.push_back(cellToPose(cell, path.header, 0.0));
  }
  assignPathOrientations(&path, goal);
  return path;
}

unsigned int CenterlinePlanner::toIndex(unsigned int x, unsigned int y) const
{
  return y * width_ + x;
}

CenterlinePlanner::Cell CenterlinePlanner::toCell(unsigned int index) const
{
  return Cell{index % width_, index / width_};
}

bool CenterlinePlanner::isObstacleCost(unsigned char cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return !allow_unknown_;
  }
  return cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
}

bool CenterlinePlanner::isTraversable(unsigned int index, float required_clearance) const
{
  if (index >= width_ * height_) {
    return false;
  }
  const Cell cell = toCell(index);
  const unsigned char cost = costmap_->getCost(cell.x, cell.y);
  if (isObstacleCost(cost)) {
    return false;
  }
  const float conservative_clearance =
    required_clearance + static_cast<float>(0.5 * resolution_);
  return clearance_m_.empty() || clearance_m_[index] >= conservative_clearance;
}

double CenterlinePlanner::traversalCost(
  unsigned int /*from_index*/, unsigned int to_index, double step_m) const
{
  const Cell cell = toCell(to_index);
  const unsigned char raw_cost = costmap_->getCost(cell.x, cell.y);

  double normalized_cost = 0.0;
  if (raw_cost == nav2_costmap_2d::NO_INFORMATION) {
    normalized_cost = 1.0;
  } else {
    normalized_cost = static_cast<double>(raw_cost) /
      static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }
  normalized_cost = std::clamp(normalized_cost, 0.0, 1.0);

  const double clearance = clearance_m_.empty() ? target_clearance_ : clearance_m_[to_index];
  const double deficit = std::max(0.0, target_clearance_ - clearance);
  const double wall_penalty =
    target_clearance_ > 0.0 ? (deficit / target_clearance_) * (deficit / target_clearance_) : 0.0;

  const double center_cap = std::max(center_clearance_cap_, target_clearance_);
  const double center_deficit = std::max(0.0, center_cap - std::min(clearance, center_cap));
  const double center_penalty =
    center_cap > 0.0 ? (center_deficit / center_cap) * (center_deficit / center_cap) : 0.0;

  return step_m * (
    length_weight_ +
    wall_weight_ * wall_penalty +
    center_weight_ * center_penalty +
    costmap_weight_ * normalized_cost);
}

double CenterlinePlanner::sequenceCost(const std::vector<Cell> & cells) const
{
  if (cells.size() < 2) {
    return 0.0;
  }
  return sequenceCost(cells, 0, cells.size() - 1);
}

double CenterlinePlanner::sequenceCost(
  const std::vector<Cell> & cells, std::size_t begin, std::size_t end) const
{
  if (cells.empty() || begin >= end || end >= cells.size()) {
    return 0.0;
  }

  double total = 0.0;
  for (std::size_t i = begin; i < end; ++i) {
    const Cell & a = cells[i];
    const Cell & b = cells[i + 1];
    const unsigned int from = toIndex(a.x, a.y);
    const unsigned int to = toIndex(b.x, b.y);
    const double dx = static_cast<double>(b.x) - static_cast<double>(a.x);
    const double dy = static_cast<double>(b.y) - static_cast<double>(a.y);
    total += traversalCost(from, to, std::hypot(dx, dy) * resolution_);
  }
  return total;
}

double CenterlinePlanner::heuristic(unsigned int index, unsigned int goal_index) const
{
  const Cell a = toCell(index);
  const Cell b = toCell(goal_index);
  const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
  const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
  return length_weight_ * std::hypot(dx, dy) * resolution_;
}

std::vector<float> CenterlinePlanner::computeClearanceField() const
{
  const unsigned int total = width_ * height_;
  std::vector<float> dist(total, kInf);

  for (unsigned int y = 0; y < height_; ++y) {
    for (unsigned int x = 0; x < width_; ++x) {
      const unsigned int idx = toIndex(x, y);
      if (isObstacleCost(costmap_->getCost(x, y))) {
        dist[idx] = 0.0F;
      }
    }
  }

  const float diag = std::sqrt(2.0F);
  for (unsigned int y = 0; y < height_; ++y) {
    for (unsigned int x = 0; x < width_; ++x) {
      const unsigned int idx = toIndex(x, y);
      float best = dist[idx];
      if (x > 0) {
        best = std::min(best, dist[toIndex(x - 1, y)] + 1.0F);
      }
      if (y > 0) {
        best = std::min(best, dist[toIndex(x, y - 1)] + 1.0F);
      }
      if (x > 0 && y > 0) {
        best = std::min(best, dist[toIndex(x - 1, y - 1)] + diag);
      }
      if (x + 1 < width_ && y > 0) {
        best = std::min(best, dist[toIndex(x + 1, y - 1)] + diag);
      }
      dist[idx] = best;
    }
  }

  for (int y = static_cast<int>(height_) - 1; y >= 0; --y) {
    for (int x = static_cast<int>(width_) - 1; x >= 0; --x) {
      const unsigned int ux = static_cast<unsigned int>(x);
      const unsigned int uy = static_cast<unsigned int>(y);
      const unsigned int idx = toIndex(ux, uy);
      float best = dist[idx];
      if (ux + 1 < width_) {
        best = std::min(best, dist[toIndex(ux + 1, uy)] + 1.0F);
      }
      if (uy + 1 < height_) {
        best = std::min(best, dist[toIndex(ux, uy + 1)] + 1.0F);
      }
      if (ux + 1 < width_ && uy + 1 < height_) {
        best = std::min(best, dist[toIndex(ux + 1, uy + 1)] + diag);
      }
      if (ux > 0 && uy + 1 < height_) {
        best = std::min(best, dist[toIndex(ux - 1, uy + 1)] + diag);
      }
      dist[idx] = best;
    }
  }

  const float fallback = static_cast<float>(std::hypot(width_, height_) * resolution_);
  for (float & value : dist) {
    value = std::isfinite(value) ? value * static_cast<float>(resolution_) : fallback;
  }
  return dist;
}

CenterlinePlanner::SearchResult CenterlinePlanner::search(
  unsigned int start_index,
  unsigned int goal_index,
  float required_clearance) const
{
  struct Compare
  {
    bool operator()(const QueueEntry & a, const QueueEntry & b) const
    {
      return a.f_score > b.f_score;
    }
  };

  const unsigned int total = width_ * height_;
  std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
  std::vector<unsigned int> parent(total, total);
  std::vector<unsigned char> closed(total, 0);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Compare> open;

  g_score[start_index] = 0.0;
  open.push(QueueEntry{start_index, heuristic(start_index, goal_index), 0.0});

  const int offsets[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
  };

  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();
    if (closed[current.index]) {
      continue;
    }
    if (current.index == goal_index) {
      std::vector<Cell> cells;
      unsigned int idx = goal_index;
      while (idx != total) {
        cells.push_back(toCell(idx));
        if (idx == start_index) {
          break;
        }
        idx = parent[idx];
      }
      std::reverse(cells.begin(), cells.end());
      return SearchResult{true, cells};
    }
    closed[current.index] = 1;

    const Cell cell = toCell(current.index);
    for (const auto & offset : offsets) {
      const int nx = static_cast<int>(cell.x) + offset[0];
      const int ny = static_cast<int>(cell.y) + offset[1];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) || ny >= static_cast<int>(height_)) {
        continue;
      }
      const unsigned int next_index =
        toIndex(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
      if (closed[next_index] || !isTraversable(next_index, required_clearance)) {
        continue;
      }

      const double step_m = (offset[0] != 0 && offset[1] != 0) ?
        resolution_ * std::sqrt(2.0) : resolution_;
      const double tentative = g_score[current.index] +
        traversalCost(current.index, next_index, step_m);
      if (tentative < g_score[next_index]) {
        parent[next_index] = current.index;
        g_score[next_index] = tentative;
        open.push(QueueEntry{
          next_index,
          tentative + heuristic(next_index, goal_index),
          tentative});
      }
    }
  }

  return SearchResult{false, {}};
}

std::vector<CenterlinePlanner::Cell> CenterlinePlanner::shortcutPath(
  const std::vector<Cell> & input) const
{
  if (input.size() <= 2) {
    return input;
  }

  std::vector<Cell> output;
  output.reserve(input.size());
  std::size_t i = 0;
  output.push_back(input.front());

  while (i + 1 < input.size()) {
    std::size_t best = i + 1;
    for (std::size_t j = input.size() - 1; j > i + 1; --j) {
      if (!hasLineOfSight(input[i], input[j], static_cast<float>(shortcut_min_clearance_))) {
        continue;
      }
      const auto line = cellsOnLine(input[i], input[j]);
      const double line_cost = sequenceCost(line);
      const double original_cost = sequenceCost(input, i, j);
      if (line_cost <= original_cost * shortcut_cost_tolerance_) {
        best = j;
        break;
      }
    }
    output.push_back(input[best]);
    i = best;
  }

  return output;
}

std::vector<CenterlinePlanner::Cell> CenterlinePlanner::densifyPath(
  const std::vector<Cell> & input) const
{
  if (input.size() <= 1) {
    return input;
  }

  std::vector<Cell> output;
  for (std::size_t i = 0; i + 1 < input.size(); ++i) {
    auto segment = cellsOnLine(input[i], input[i + 1]);
    if (!output.empty() && !segment.empty()) {
      segment.erase(segment.begin());
    }
    output.insert(output.end(), segment.begin(), segment.end());
  }
  return output;
}

std::vector<CenterlinePlanner::Cell> CenterlinePlanner::cellsOnLine(
  const Cell & a, const Cell & b) const
{
  std::vector<Cell> cells;
  int x0 = static_cast<int>(a.x);
  int y0 = static_cast<int>(a.y);
  const int x1 = static_cast<int>(b.x);
  const int y1 = static_cast<int>(b.y);
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (true) {
    cells.push_back(Cell{static_cast<unsigned int>(x0), static_cast<unsigned int>(y0)});
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }

  return cells;
}

bool CenterlinePlanner::hasLineOfSight(
  const Cell & a, const Cell & b, float required_clearance) const
{
  for (const auto & cell : cellsOnLine(a, b)) {
    const unsigned int index = toIndex(cell.x, cell.y);
    if (!isTraversable(index, required_clearance)) {
      return false;
    }
  }
  return true;
}

bool CenterlinePlanner::findNearestTraversable(
  unsigned int seed_index,
  unsigned int * result_index,
  float required_clearance) const
{
  if (isTraversable(seed_index, required_clearance)) {
    *result_index = seed_index;
    return true;
  }

  const Cell seed = toCell(seed_index);
  const int max_radius = std::max(1, static_cast<int>(std::ceil(endpoint_snap_radius_ / resolution_)));
  for (int radius = 1; radius <= max_radius; ++radius) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }
        const int x = static_cast<int>(seed.x) + dx;
        const int y = static_cast<int>(seed.y) + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_)) {
          continue;
        }
        const unsigned int index =
          toIndex(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
        if (isTraversable(index, required_clearance)) {
          *result_index = index;
          return true;
        }
      }
    }
  }

  return false;
}

geometry_msgs::msg::PoseStamped CenterlinePlanner::cellToPose(
  const Cell & cell,
  const std_msgs::msg::Header & header,
  double yaw) const
{
  double wx = 0.0;
  double wy = 0.0;
  costmap_->mapToWorld(cell.x, cell.y, wx, wy);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = wx;
  pose.pose.position.y = wy;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = yawToQuaternion(yaw);
  return pose;
}

void CenterlinePlanner::assignPathOrientations(
  nav_msgs::msg::Path * path,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  if (path == nullptr || path->poses.empty()) {
    return;
  }

  for (std::size_t i = 0; i + 1 < path->poses.size(); ++i) {
    const auto & p = path->poses[i].pose.position;
    const auto & n = path->poses[i + 1].pose.position;
    path->poses[i].pose.orientation = yawToQuaternion(yawFromDelta(n.x - p.x, n.y - p.y));
  }

  if (use_final_approach_orientation_ && path->poses.size() >= 2) {
    const auto & p = path->poses[path->poses.size() - 2].pose.position;
    const auto & n = path->poses.back().pose.position;
    path->poses.back().pose.orientation = yawToQuaternion(yawFromDelta(n.x - p.x, n.y - p.y));
  } else {
    path->poses.back().pose.orientation = goal.pose.orientation;
  }
}

}  // namespace g1_centerline_planner

PLUGINLIB_EXPORT_CLASS(g1_centerline_planner::CenterlinePlanner, nav2_core::GlobalPlanner)
