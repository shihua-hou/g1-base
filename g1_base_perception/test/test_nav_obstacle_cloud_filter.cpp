#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <vector>

#include "g1_base_perception/nav_obstacle_cloud_filter.hpp"

namespace
{

std::vector<Eigen::Vector3d> floorAndObstacles()
{
  std::vector<Eigen::Vector3d> points;
  for (int ix = -10; ix <= 10; ++ix) {
    for (int iy = -10; iy <= 10; ++iy) {
      const double x = static_cast<double>(ix) * 0.2;
      const double y = static_cast<double>(iy) * 0.2;
      if (std::hypot(x, y) >= 0.5) {
        points.emplace_back(x, y, 0.0);
      }
    }
  }

  points.emplace_back(1.0, 0.0, 0.35);
  points.emplace_back(1.1, 0.0, 0.35);
  points.emplace_back(1.2, 0.0, 0.35);
  return points;
}

g1_base_perception::FilterConfig testConfig()
{
  g1_base_perception::FilterConfig config;
  config.ground_min_inliers = 20;
  config.ground_fit_min_range = 0.0;
  config.ground_fit_max_range = 4.0;
  config.voxel_leaf_size = 0.0;
  return config;
}

}  // namespace

TEST(NavObstacleCloudFilterCore, ImuPriorSeedsGroundFitAndFiltersObstacles)
{
  g1_base_perception::NavObstacleCloudFilterCore core(testConfig());
  const auto cloud = floorAndObstacles();
  const auto prior = core.gravityPriorNormal(
    Eigen::Vector3d(0.0, 0.0, -1.0), Eigen::Quaterniond::Identity());

  ASSERT_TRUE(prior.has_value());
  const auto fit = core.fitGroundPlane(cloud, Eigen::Vector3d(0.0, 0.0, 1.0), prior);

  ASSERT_TRUE(fit.has_value());
  EXPECT_TRUE(fit->used_prior);
  EXPECT_EQ(fit->iterations, 0);

  core.acceptGroundPlane(fit->plane.point, fit->plane.normal);
  const auto output = core.filterObstacles(cloud, Eigen::Vector3d(0.0, 0.0, 1.0));

  EXPECT_EQ(output.size(), 3u);
}

TEST(NavObstacleCloudFilterCore, ReusesStablePlaneUntilFailureLimit)
{
  g1_base_perception::NavObstacleCloudFilterCore core(testConfig());
  const auto cloud = floorAndObstacles();

  core.acceptGroundPlane(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0));
  core.recordGroundFitFailure();

  EXPECT_EQ(core.consecutiveFailCount(), 1);
  EXPECT_FALSE(core.filterObstacles(cloud, Eigen::Vector3d(0.0, 0.0, 1.0)).empty());

  for (int i = 0; i < 30; ++i) {
    core.recordGroundFitFailure();
  }

  EXPECT_EQ(core.consecutiveFailCount(), 31);
  EXPECT_TRUE(core.filterObstacles(cloud, Eigen::Vector3d(0.0, 0.0, 1.0)).empty());
}

TEST(NavObstacleCloudFilterCore, VoxelFilterUsesCentroidsAndMinimumCounts)
{
  auto config = testConfig();
  config.voxel_leaf_size = 0.5;
  config.voxel_min_points = 2;
  g1_base_perception::NavObstacleCloudFilterCore core(config);

  const std::vector<Eigen::Vector3f> points = {
    Eigen::Vector3f(1.00F, 0.00F, 0.35F),
    Eigen::Vector3f(1.10F, 0.10F, 0.45F),
    Eigen::Vector3f(2.00F, 0.00F, 0.50F),
  };

  const auto output = core.voxelFilter(points);

  ASSERT_EQ(output.size(), 1u);
  EXPECT_NEAR(output[0].x(), 1.05F, 1e-5F);
  EXPECT_NEAR(output[0].y(), 0.05F, 1e-5F);
  EXPECT_NEAR(output[0].z(), 0.40F, 1e-5F);
}
