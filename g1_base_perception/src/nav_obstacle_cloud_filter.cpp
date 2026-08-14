#include "g1_base_perception/nav_obstacle_cloud_filter.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"

namespace g1_base_perception
{
namespace
{

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

struct VoxelKey
{
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<std::int64_t>{}(key.x);
    const auto hy = std::hash<std::int64_t>{}(key.y);
    const auto hz = std::hash<std::int64_t>{}(key.z);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

struct FieldInfo
{
  std::uint32_t offset = 0;
  std::uint8_t datatype = 0;
};

bool hostIsBigEndian()
{
  const std::uint16_t value = 0x0102;
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&value);
  return bytes[0] == 0x01;
}

template<typename T>
T byteSwap(T value)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  std::reverse(bytes.begin(), bytes.end());
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

std::optional<FieldInfo> findField(const PointCloud2 & msg, const char * name)
{
  for (const auto & field : msg.fields) {
    if (field.name == name) {
      return FieldInfo{field.offset, field.datatype};
    }
  }
  return std::nullopt;
}

std::optional<double> readPointValue(
  const PointCloud2 & msg,
  const FieldInfo & field,
  std::size_t point_offset)
{
  const std::size_t offset = point_offset + field.offset;
  const bool swap = msg.is_bigendian != hostIsBigEndian();

  if (field.datatype == PointField::FLOAT32) {
    if (offset + sizeof(float) > msg.data.size()) {
      return std::nullopt;
    }
    float value = 0.0F;
    std::memcpy(&value, msg.data.data() + offset, sizeof(float));
    if (swap) {
      value = byteSwap(value);
    }
    return static_cast<double>(value);
  }

  if (field.datatype == PointField::FLOAT64) {
    if (offset + sizeof(double) > msg.data.size()) {
      return std::nullopt;
    }
    double value = 0.0;
    std::memcpy(&value, msg.data.data() + offset, sizeof(double));
    if (swap) {
      value = byteSwap(value);
    }
    return value;
  }

  return std::nullopt;
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

}  // namespace

NavObstacleCloudFilterCore::NavObstacleCloudFilterCore(FilterConfig config)
: config_(std::move(config)), rng_(42)
{
}

FilterConfig & NavObstacleCloudFilterCore::config()
{
  return config_;
}

const FilterConfig & NavObstacleCloudFilterCore::config() const
{
  return config_;
}

void NavObstacleCloudFilterCore::setConfig(const FilterConfig & config)
{
  config_ = config;
}

std::optional<Eigen::Vector3d> NavObstacleCloudFilterCore::normalized(
  const Eigen::Vector3d & vector) const
{
  const double norm = vector.norm();
  if (!std::isfinite(norm) || norm < 1e-9) {
    return std::nullopt;
  }
  return vector / norm;
}

std::optional<Eigen::Vector3d> NavObstacleCloudFilterCore::gravityPriorNormal(
  const Eigen::Vector3d & imu_accel,
  const Eigen::Quaterniond & body_to_world) const
{
  const auto gravity_body = normalized(-imu_accel);
  if (!gravity_body.has_value()) {
    return std::nullopt;
  }

  const double quat_norm = body_to_world.norm();
  if (!std::isfinite(quat_norm) || quat_norm < 1e-9) {
    return std::nullopt;
  }
  const Eigen::Quaterniond q = body_to_world.normalized();
  return normalized(q * gravity_body.value());
}

std::optional<GroundFitResult> NavObstacleCloudFilterCore::fitGroundPlane(
  const std::vector<Eigen::Vector3d> & cloud,
  const Eigen::Vector3d & odom_xyz,
  const std::optional<Eigen::Vector3d> & prior_normal_input)
{
  const auto prior_normal = prior_normal_input.has_value() ?
    normalized(prior_normal_input.value()) : std::optional<Eigen::Vector3d>{};
  const double prior_angle = clamp(config_.ground_prior_max_angle_deg, 0.0, 89.0);
  const double prior_cos = std::cos(prior_angle * M_PI / 180.0);

  std::vector<Eigen::Vector3d> candidates;
  candidates.reserve(cloud.size());
  for (const auto & point : cloud) {
    const double range_xy = std::hypot(point.x() - odom_xyz.x(), point.y() - odom_xyz.y());
    if (range_xy >= config_.ground_fit_min_range && range_xy <= config_.ground_fit_max_range) {
      candidates.push_back(point);
    }
  }

  const auto min_candidate_count = static_cast<std::size_t>(std::max(3, config_.ground_min_inliers));
  if (candidates.size() < min_candidate_count) {
    return std::nullopt;
  }

  GroundFitResult result;
  result.used_prior = prior_normal.has_value();

  if (prior_normal.has_value()) {
    const auto seeded = fitGroundPlaneFromPrior(candidates, odom_xyz, prior_normal.value());
    if (seeded.has_value()) {
      result.plane = seeded.value();
      result.iterations = 0;
      result.used_prior = true;
      return result;
    }
  }

  std::vector<bool> best_inliers;
  int best_count = 0;
  const int iterations = std::max(config_.ground_ransac_iterations, 1);
  std::uniform_int_distribution<std::size_t> dist(0, candidates.size() - 1);

  for (int iteration = 0; iteration < iterations; ++iteration) {
    result.iterations += 1;
    const std::size_t i0 = dist(rng_);
    std::size_t i1 = dist(rng_);
    std::size_t i2 = dist(rng_);
    for (int guard = 0; guard < 12 && i1 == i0; ++guard) {
      i1 = dist(rng_);
    }
    for (int guard = 0; guard < 12 && (i2 == i0 || i2 == i1); ++guard) {
      i2 = dist(rng_);
    }
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }

    const auto & p0 = candidates[i0];
    const auto & p1 = candidates[i1];
    const auto & p2 = candidates[i2];
    Eigen::Vector3d normal = (p1 - p0).cross(p2 - p0);
    const double norm = normal.norm();
    if (norm < 1e-6) {
      continue;
    }
    normal /= norm;

    if (prior_normal.has_value()) {
      double prior_dot = normal.dot(prior_normal.value());
      if (prior_dot < 0.0) {
        normal = -normal;
        prior_dot = -prior_dot;
      }
      if (prior_dot < prior_cos) {
        continue;
      }
    } else if (std::abs(normal.z()) < config_.ground_min_normal_z) {
      continue;
    }

    std::vector<bool> inliers(candidates.size(), false);
    int count = 0;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const double distance = std::abs((candidates[i] - p0).dot(normal));
      if (distance <= config_.ground_inlier_distance) {
        inliers[i] = true;
        count += 1;
      }
    }

    if (count > best_count) {
      best_count = count;
      best_inliers = std::move(inliers);
    }
  }

  if (best_inliers.empty() || best_count < config_.ground_min_inliers) {
    return std::nullopt;
  }

  std::vector<Eigen::Vector3d> inlier_points;
  inlier_points.reserve(static_cast<std::size_t>(best_count));
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (best_inliers[i]) {
      inlier_points.push_back(candidates[i]);
    }
  }

  const auto refined = refineGroundPlane(inlier_points, odom_xyz, prior_normal);
  if (!refined.has_value()) {
    return std::nullopt;
  }
  result.plane = refined.value();
  result.used_prior = prior_normal.has_value();
  return result;
}

bool NavObstacleCloudFilterCore::acceptGroundPlane(
  const Eigen::Vector3d & plane_point,
  const Eigen::Vector3d & normal)
{
  const auto normalized_normal = normalized(normal);
  if (!normalized_normal.has_value()) {
    recordGroundFitFailure();
    return false;
  }

  const double alpha = clamp(config_.ground_ema_alpha, 0.0, 1.0);
  GroundPlane accepted;
  if (!stable_ground_plane_.has_value()) {
    accepted.point = plane_point;
    accepted.normal = normalized_normal.value();
  } else {
    auto normal_oriented = normalized_normal.value();
    if (stable_ground_plane_->normal.dot(normal_oriented) < 0.0) {
      normal_oriented = -normal_oriented;
    }
    accepted.point = (1.0 - alpha) * stable_ground_plane_->point + alpha * plane_point;
    const auto smoothed_normal = normalized(
      (1.0 - alpha) * stable_ground_plane_->normal + alpha * normal_oriented);
    accepted.normal = smoothed_normal.value_or(stable_ground_plane_->normal);
  }

  stable_ground_plane_ = accepted;
  consecutive_fail_count_ = 0;
  return true;
}

void NavObstacleCloudFilterCore::recordGroundFitFailure()
{
  consecutive_fail_count_ += 1;
}

int NavObstacleCloudFilterCore::consecutiveFailCount() const
{
  return consecutive_fail_count_;
}

bool NavObstacleCloudFilterCore::hasUsableGroundPlane() const
{
  return stable_ground_plane_.has_value() && consecutive_fail_count_ <= kMaxGroundFitFailures;
}

std::vector<Eigen::Vector3f> NavObstacleCloudFilterCore::filterObstacles(
  const std::vector<Eigen::Vector3d> & cloud,
  const Eigen::Vector3d & odom_xyz) const
{
  if (!hasUsableGroundPlane()) {
    return {};
  }

  std::vector<Eigen::Vector3f> selected;
  selected.reserve(cloud.size());
  const auto & plane = stable_ground_plane_.value();
  for (const auto & point : cloud) {
    const double range_xy = std::hypot(point.x() - odom_xyz.x(), point.y() - odom_xyz.y());
    const double height = (point - plane.point).dot(plane.normal);
    if (
      range_xy >= config_.min_range &&
      range_xy <= config_.max_range &&
      height >= config_.obstacle_min_height &&
      height <= config_.obstacle_max_height)
    {
      selected.push_back(point.cast<float>());
    }
  }

  return voxelFilter(selected);
}

std::vector<Eigen::Vector3f> NavObstacleCloudFilterCore::voxelFilter(
  const std::vector<Eigen::Vector3f> & points) const
{
  if (points.empty() || config_.voxel_leaf_size <= 0.0) {
    return points;
  }

  struct Accum
  {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int count = 0;
  };

  std::unordered_map<VoxelKey, Accum, VoxelKeyHash> voxels;
  for (const auto & point : points) {
    const VoxelKey key{
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.x()) / config_.voxel_leaf_size)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.y()) / config_.voxel_leaf_size)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.z()) / config_.voxel_leaf_size))};
    auto & accum = voxels[key];
    accum.sum += point.cast<double>();
    accum.count += 1;
  }

  std::vector<Eigen::Vector3f> filtered;
  filtered.reserve(voxels.size());
  for (const auto & entry : voxels) {
    const auto & accum = entry.second;
    if (accum.count >= config_.voxel_min_points) {
      filtered.push_back((accum.sum / static_cast<double>(accum.count)).cast<float>());
    }
  }
  return filtered;
}

std::optional<GroundPlane> NavObstacleCloudFilterCore::fitGroundPlaneFromPrior(
  const std::vector<Eigen::Vector3d> & candidates,
  const Eigen::Vector3d & odom_xyz,
  const Eigen::Vector3d & prior_normal) const
{
  std::vector<double> projections;
  projections.reserve(candidates.size());
  for (const auto & point : candidates) {
    projections.push_back(point.dot(prior_normal));
  }

  const double seed_distance = percentile10(projections);
  std::vector<Eigen::Vector3d> inliers;
  inliers.reserve(candidates.size());
  for (const auto & point : candidates) {
    if (std::abs(point.dot(prior_normal) - seed_distance) <= config_.ground_inlier_distance) {
      inliers.push_back(point);
    }
  }

  if (static_cast<int>(inliers.size()) < config_.ground_min_inliers) {
    return std::nullopt;
  }
  return refineGroundPlane(inliers, odom_xyz, prior_normal);
}

std::optional<GroundPlane> NavObstacleCloudFilterCore::refineGroundPlane(
  const std::vector<Eigen::Vector3d> & inlier_points,
  const Eigen::Vector3d & odom_xyz,
  const std::optional<Eigen::Vector3d> & prior_normal) const
{
  if (inlier_points.size() < 3) {
    return std::nullopt;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & point : inlier_points) {
    centroid += point;
  }
  centroid /= static_cast<double>(inlier_points.size());

  Eigen::MatrixXd centered(inlier_points.size(), 3);
  for (std::size_t i = 0; i < inlier_points.size(); ++i) {
    centered.row(static_cast<Eigen::Index>(i)) = inlier_points[i] - centroid;
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeFullV);
  Eigen::Vector3d normal = svd.matrixV().col(2);
  const double norm = normal.norm();
  if (norm < 1e-6) {
    return std::nullopt;
  }
  normal /= norm;

  if (prior_normal.has_value()) {
    if (normal.dot(prior_normal.value()) < 0.0) {
      normal = -normal;
    }
    const double prior_angle = clamp(config_.ground_prior_max_angle_deg, 0.0, 89.0);
    const double prior_cos = std::cos(prior_angle * M_PI / 180.0);
    if (normal.dot(prior_normal.value()) < prior_cos) {
      return std::nullopt;
    }
  } else if (std::abs(normal.z()) < config_.ground_min_normal_z) {
    return std::nullopt;
  } else if ((odom_xyz - centroid).dot(normal) < 0.0) {
    normal = -normal;
  }

  return GroundPlane{centroid, normal};
}

double NavObstacleCloudFilterCore::percentile10(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double rank = 0.10 * static_cast<double>(values.size() - 1U);
  const auto lo = static_cast<std::size_t>(std::floor(rank));
  const auto hi = static_cast<std::size_t>(std::ceil(rank));
  const double fraction = rank - static_cast<double>(lo);
  return values[lo] * (1.0 - fraction) + values[hi] * fraction;
}

NavObstacleCloudFilter::NavObstacleCloudFilter()
: Node("nav_obstacle_cloud_filter"), core_(FilterConfig{})
{
  declareParameters();

  input_cloud_topic_ = get_parameter("input_cloud_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  imu_topic_ = get_parameter("imu_topic").as_string();
  output_cloud_topic_ = get_parameter("output_cloud_topic").as_string();
  output_frame_ = get_parameter("output_frame").as_string();
  core_.setConfig(readConfig());

  auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, qos, std::bind(&NavObstacleCloudFilter::onOdom, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, qos, std::bind(&NavObstacleCloudFilter::onImu, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, qos, std::bind(&NavObstacleCloudFilter::onCloud, this, std::placeholders::_1));
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);

  ground_fit_timer_ = create_wall_timer(
    std::chrono::duration<double>(std::max(0.1, core_.config().ground_fit_period_sec)),
    std::bind(&NavObstacleCloudFilter::onGroundFitTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "nav_obstacle_cloud_filter started: %s + %s + %s -> %s (%s)",
    input_cloud_topic_.c_str(),
    odom_topic_.c_str(),
    imu_topic_.c_str(),
    output_cloud_topic_.c_str(),
    output_frame_.c_str());
}

void NavObstacleCloudFilter::declareParameters()
{
  declare_parameter<std::string>("input_cloud_topic", "/lio/cloud_world");
  declare_parameter<std::string>("odom_topic", "/lio/robo/odom");
  declare_parameter<std::string>("imu_topic", "/livox/imu");
  declare_parameter<std::string>("output_cloud_topic", "/nav/obstacle_cloud");
  declare_parameter<std::string>("output_frame", "world");

  declare_parameter<double>("min_range", 0.45);
  declare_parameter<double>("max_range", 3.0);
  declare_parameter<double>("ground_fit_min_range", 0.7);
  declare_parameter<double>("ground_fit_max_range", 4.0);
  declare_parameter<double>("obstacle_min_height", 0.15);
  declare_parameter<double>("obstacle_max_height", 1.6);

  declare_parameter<int>("ground_ransac_iterations", 72);
  declare_parameter<double>("ground_inlier_distance", 0.10);
  declare_parameter<int>("ground_min_inliers", 80);
  declare_parameter<double>("ground_min_normal_z", 0.55);
  declare_parameter<double>("ground_fit_period_sec", 1.0);
  declare_parameter<double>("ground_ema_alpha", 0.25);
  declare_parameter<double>("ground_prior_max_angle_deg", 25.0);
  declare_parameter<double>("imu_max_age", 1.0);

  declare_parameter<double>("voxel_leaf_size", 0.08);
  declare_parameter<int>("voxel_min_points", 1);
  declare_parameter<double>("max_odom_age", 0.5);
  declare_parameter<bool>("publish_empty_on_failure", true);
}

FilterConfig NavObstacleCloudFilter::readConfig() const
{
  FilterConfig config;
  config.min_range = get_parameter("min_range").as_double();
  config.max_range = get_parameter("max_range").as_double();
  config.ground_fit_min_range = get_parameter("ground_fit_min_range").as_double();
  config.ground_fit_max_range = get_parameter("ground_fit_max_range").as_double();
  config.obstacle_min_height = get_parameter("obstacle_min_height").as_double();
  config.obstacle_max_height = get_parameter("obstacle_max_height").as_double();
  config.ground_ransac_iterations = static_cast<int>(get_parameter("ground_ransac_iterations").as_int());
  config.ground_inlier_distance = get_parameter("ground_inlier_distance").as_double();
  config.ground_min_inliers = static_cast<int>(get_parameter("ground_min_inliers").as_int());
  config.ground_min_normal_z = get_parameter("ground_min_normal_z").as_double();
  config.ground_fit_period_sec = get_parameter("ground_fit_period_sec").as_double();
  config.ground_ema_alpha = get_parameter("ground_ema_alpha").as_double();
  config.ground_prior_max_angle_deg = get_parameter("ground_prior_max_angle_deg").as_double();
  config.imu_max_age = get_parameter("imu_max_age").as_double();
  config.voxel_leaf_size = get_parameter("voxel_leaf_size").as_double();
  config.voxel_min_points = static_cast<int>(get_parameter("voxel_min_points").as_int());
  config.max_odom_age = get_parameter("max_odom_age").as_double();
  config.publish_empty_on_failure = get_parameter("publish_empty_on_failure").as_bool();
  return config;
}

void NavObstacleCloudFilter::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & pose = msg->pose.pose;
  const std::array<double, 7> values = {
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
  if (!std::all_of(values.begin(), values.end(), finite)) {
    warnThrottled("invalid_odom", "ignoring invalid odometry");
    return;
  }
  latest_odom_ = msg;
  latest_odom_time_ = std::chrono::steady_clock::now();
}

void NavObstacleCloudFilter::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto & accel = msg->linear_acceleration;
  if (!finite(accel.x) || !finite(accel.y) || !finite(accel.z)) {
    warnThrottled("invalid_imu", "ignoring invalid IMU acceleration");
    return;
  }
  const Eigen::Vector3d accel_vec(accel.x, accel.y, accel.z);
  if (accel_vec.norm() < 1e-6) {
    warnThrottled("invalid_imu", "ignoring near-zero IMU acceleration");
    return;
  }
  latest_imu_accel_ = accel_vec;
  latest_imu_time_ = std::chrono::steady_clock::now();
}

void NavObstacleCloudFilter::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  core_.setConfig(readConfig());

  if (!latest_odom_) {
    warnThrottled("no_odom", "no odometry yet; publishing empty cloud");
    publishEmpty(*msg);
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double odom_age = std::chrono::duration<double>(now - latest_odom_time_).count();
  if (odom_age > core_.config().max_odom_age) {
    std::ostringstream message;
    message << "odometry age " << std::fixed << std::setprecision(2) << odom_age
            << "s exceeds " << core_.config().max_odom_age << "s; publishing empty cloud";
    warnThrottled("stale_odom", message.str());
    publishEmpty(*msg);
    return;
  }

  const auto cloud = cloudToPoints(*msg);
  if (cloud.empty()) {
    publishEmpty(*msg);
    return;
  }

  latest_cloud_ = cloud;
  have_latest_cloud_ = true;
  const auto output = core_.filterObstacles(cloud, odomPosition(*latest_odom_));
  if (output.empty() && !core_.hasUsableGroundPlane()) {
    warnThrottled("ground_fit", "no stable ground plane available; publishing empty cloud");
  }
  publishPoints(output, *msg);
}

void NavObstacleCloudFilter::onGroundFitTimer()
{
  core_.setConfig(readConfig());

  if (!latest_odom_ || !have_latest_cloud_ || latest_cloud_.empty()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double odom_age = std::chrono::duration<double>(now - latest_odom_time_).count();
  if (odom_age > core_.config().max_odom_age) {
    return;
  }

  const auto prior_normal = gravityPriorNormal(*latest_odom_);
  if (!prior_normal.has_value()) {
    if (!core_.hasUsableGroundPlane()) {
      warnThrottled(
        "no_ground_prior",
        "no IMU/odom gravity prior yet; waiting to fit ground plane",
        5.0);
    }
    return;
  }

  const auto ground_plane = core_.fitGroundPlane(
    latest_cloud_, odomPosition(*latest_odom_), prior_normal);
  if (!ground_plane.has_value()) {
    core_.recordGroundFitFailure();
    if (core_.consecutiveFailCount() > 30) {
      warnThrottled(
        "ground_fit_forced_empty",
        "ground plane fit failed repeatedly; publishing empty obstacle clouds",
        2.0);
    } else if (core_.consecutiveFailCount() > 5) {
      warnThrottled(
        "ground_fit_reuse",
        "ground plane fit failed repeatedly; reusing the last stable plane",
        5.0);
    }
    return;
  }

  const int previous_failures = core_.consecutiveFailCount();
  core_.acceptGroundPlane(ground_plane->plane.point, ground_plane->plane.normal);
  if (previous_failures > 0) {
    RCLCPP_INFO(
      get_logger(),
      "ground plane fit recovered after %d consecutive failures",
      previous_failures);
  }
}

std::optional<Eigen::Vector3d> NavObstacleCloudFilter::gravityPriorNormal(
  const nav_msgs::msg::Odometry & odom) const
{
  if (!latest_imu_accel_.has_value()) {
    return std::nullopt;
  }

  const auto now = std::chrono::steady_clock::now();
  const double imu_age = std::chrono::duration<double>(now - latest_imu_time_).count();
  if (imu_age > core_.config().imu_max_age) {
    return std::nullopt;
  }

  return core_.gravityPriorNormal(latest_imu_accel_.value(), odomOrientation(odom));
}

void NavObstacleCloudFilter::publishEmpty(const sensor_msgs::msg::PointCloud2 & source_msg)
{
  if (core_.config().publish_empty_on_failure) {
    publishPoints({}, source_msg);
  }
}

void NavObstacleCloudFilter::publishPoints(
  const std::vector<Eigen::Vector3f> & points,
  const sensor_msgs::msg::PointCloud2 & source_msg)
{
  (void)source_msg;
  std_msgs::msg::Header header;
  header.stamp = get_clock()->now();
  header.frame_id = output_frame_;
  cloud_pub_->publish(makeCloud(points, header));
}

void NavObstacleCloudFilter::warnThrottled(
  const std::string & key,
  const std::string & message,
  double period_sec)
{
  const auto now = std::chrono::steady_clock::now();
  const auto found = warn_times_.find(key);
  if (
    found == warn_times_.end() ||
    std::chrono::duration<double>(now - found->second).count() >= period_sec)
  {
    warn_times_[key] = now;
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
}

bool NavObstacleCloudFilter::finite(double value)
{
  return std::isfinite(value);
}

Eigen::Vector3d NavObstacleCloudFilter::odomPosition(const nav_msgs::msg::Odometry & odom)
{
  const auto & pos = odom.pose.pose.position;
  return Eigen::Vector3d(pos.x, pos.y, pos.z);
}

Eigen::Quaterniond NavObstacleCloudFilter::odomOrientation(const nav_msgs::msg::Odometry & odom)
{
  const auto & q = odom.pose.pose.orientation;
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

std::vector<Eigen::Vector3d> NavObstacleCloudFilter::cloudToPoints(
  const sensor_msgs::msg::PointCloud2 & msg)
{
  const auto x_field = findField(msg, "x");
  const auto y_field = findField(msg, "y");
  const auto z_field = findField(msg, "z");
  if (!x_field.has_value() || !y_field.has_value() || !z_field.has_value()) {
    return {};
  }

  std::vector<Eigen::Vector3d> points;
  points.reserve(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height));
  for (std::uint32_t row = 0; row < msg.height; ++row) {
    for (std::uint32_t col = 0; col < msg.width; ++col) {
      const std::size_t point_offset =
        static_cast<std::size_t>(row) * msg.row_step +
        static_cast<std::size_t>(col) * msg.point_step;
      const auto x = readPointValue(msg, x_field.value(), point_offset);
      const auto y = readPointValue(msg, y_field.value(), point_offset);
      const auto z = readPointValue(msg, z_field.value(), point_offset);
      if (
        x.has_value() && y.has_value() && z.has_value() &&
        finite(x.value()) && finite(y.value()) && finite(z.value()))
      {
        points.emplace_back(x.value(), y.value(), z.value());
      }
    }
  }
  return points;
}

sensor_msgs::msg::PointCloud2 NavObstacleCloudFilter::makeCloud(
  const std::vector<Eigen::Vector3f> & points,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = 12;
  msg.row_step = msg.point_step * msg.width;
  msg.fields.resize(3);

  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.data.resize(static_cast<std::size_t>(msg.row_step));
  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t offset = i * msg.point_step;
    const float values[3] = {points[i].x(), points[i].y(), points[i].z()};
    std::memcpy(msg.data.data() + offset, values, sizeof(values));
  }
  return msg;
}

}  // namespace g1_base_perception
