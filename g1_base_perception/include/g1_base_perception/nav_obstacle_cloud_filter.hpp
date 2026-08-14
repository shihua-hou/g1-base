#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace g1_base_perception
{

struct FilterConfig
{
  double min_range = 0.45;
  double max_range = 3.0;
  double ground_fit_min_range = 0.7;
  double ground_fit_max_range = 4.0;
  double obstacle_min_height = 0.15;
  double obstacle_max_height = 1.6;
  int ground_ransac_iterations = 72;
  double ground_inlier_distance = 0.10;
  int ground_min_inliers = 80;
  double ground_min_normal_z = 0.55;
  double ground_fit_period_sec = 1.0;
  double ground_ema_alpha = 0.25;
  double ground_prior_max_angle_deg = 25.0;
  double imu_max_age = 1.0;
  double voxel_leaf_size = 0.08;
  int voxel_min_points = 1;
  double max_odom_age = 0.5;
  bool publish_empty_on_failure = true;
};

struct GroundPlane
{
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

struct GroundFitResult
{
  GroundPlane plane;
  int iterations = 0;
  bool used_prior = false;
};

class NavObstacleCloudFilterCore
{
public:
  explicit NavObstacleCloudFilterCore(FilterConfig config = {});

  FilterConfig & config();
  const FilterConfig & config() const;
  void setConfig(const FilterConfig & config);

  std::optional<Eigen::Vector3d> normalized(const Eigen::Vector3d & vector) const;
  std::optional<Eigen::Vector3d> gravityPriorNormal(
    const Eigen::Vector3d & imu_accel,
    const Eigen::Quaterniond & body_to_world) const;

  std::optional<GroundFitResult> fitGroundPlane(
    const std::vector<Eigen::Vector3d> & cloud,
    const Eigen::Vector3d & odom_xyz,
    const std::optional<Eigen::Vector3d> & prior_normal);

  bool acceptGroundPlane(const Eigen::Vector3d & plane_point, const Eigen::Vector3d & normal);
  void recordGroundFitFailure();
  int consecutiveFailCount() const;
  bool hasUsableGroundPlane() const;

  std::vector<Eigen::Vector3f> filterObstacles(
    const std::vector<Eigen::Vector3d> & cloud,
    const Eigen::Vector3d & odom_xyz) const;

  std::vector<Eigen::Vector3f> voxelFilter(
    const std::vector<Eigen::Vector3f> & points) const;

private:
  std::optional<GroundPlane> fitGroundPlaneFromPrior(
    const std::vector<Eigen::Vector3d> & candidates,
    const Eigen::Vector3d & odom_xyz,
    const Eigen::Vector3d & prior_normal) const;

  std::optional<GroundPlane> refineGroundPlane(
    const std::vector<Eigen::Vector3d> & inlier_points,
    const Eigen::Vector3d & odom_xyz,
    const std::optional<Eigen::Vector3d> & prior_normal) const;

  static double percentile10(std::vector<double> values);

  FilterConfig config_;
  std::optional<GroundPlane> stable_ground_plane_;
  int consecutive_fail_count_ = 0;
  static constexpr int kMaxGroundFitFailures = 30;
  std::mt19937 rng_;
};

class NavObstacleCloudFilter : public rclcpp::Node
{
public:
  NavObstacleCloudFilter();

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void declareParameters();
  FilterConfig readConfig() const;

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onGroundFitTimer();

  std::optional<Eigen::Vector3d> gravityPriorNormal(const nav_msgs::msg::Odometry & odom) const;
  void publishEmpty(const sensor_msgs::msg::PointCloud2 & source_msg);
  void publishPoints(
    const std::vector<Eigen::Vector3f> & points,
    const sensor_msgs::msg::PointCloud2 & source_msg);
  void warnThrottled(
    const std::string & key,
    const std::string & message,
    double period_sec = 2.0);

  static bool finite(double value);
  static Eigen::Vector3d odomPosition(const nav_msgs::msg::Odometry & odom);
  static Eigen::Quaterniond odomOrientation(const nav_msgs::msg::Odometry & odom);
  static std::vector<Eigen::Vector3d> cloudToPoints(
    const sensor_msgs::msg::PointCloud2 & msg);
  static sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<Eigen::Vector3f> & points,
    const std_msgs::msg::Header & header);

  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string output_cloud_topic_;
  std::string output_frame_;

  NavObstacleCloudFilterCore core_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  SteadyTime latest_odom_time_{};
  std::optional<Eigen::Vector3d> latest_imu_accel_;
  SteadyTime latest_imu_time_{};
  std::vector<Eigen::Vector3d> latest_cloud_;
  bool have_latest_cloud_ = false;
  std::unordered_map<std::string, SteadyTime> warn_times_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr ground_fit_timer_;
};

}  // namespace g1_base_perception
