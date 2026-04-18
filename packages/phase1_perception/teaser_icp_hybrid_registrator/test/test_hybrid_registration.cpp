#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "teaser_icp_hybrid/hybrid_registrator.hpp"

namespace perspective_grasp {
namespace {

/// Generate a synthetic cube point cloud centered at origin
pcl::PointCloud<pcl::PointXYZ>::Ptr makeCubeCloud(
    double side_length, int points_per_face) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  double half = side_length / 2.0;
  double step = side_length / static_cast<double>(points_per_face - 1);

  // Generate points on 6 faces
  for (int i = 0; i < points_per_face; ++i) {
    for (int j = 0; j < points_per_face; ++j) {
      double u = -half + step * static_cast<double>(i);
      double v = -half + step * static_cast<double>(j);
      cloud->push_back(pcl::PointXYZ(
          static_cast<float>(u), static_cast<float>(v), static_cast<float>(half)));
      cloud->push_back(pcl::PointXYZ(
          static_cast<float>(u), static_cast<float>(v), static_cast<float>(-half)));
      cloud->push_back(pcl::PointXYZ(
          static_cast<float>(u), static_cast<float>(half), static_cast<float>(v)));
      cloud->push_back(pcl::PointXYZ(
          static_cast<float>(u), static_cast<float>(-half), static_cast<float>(v)));
      cloud->push_back(pcl::PointXYZ(
          static_cast<float>(half), static_cast<float>(u), static_cast<float>(v)));
      cloud->push_back(pcl::PointXYZ(
          static_cast<float>(-half), static_cast<float>(u), static_cast<float>(v)));
    }
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

TEST(HybridRegistrator, IcpRefineKnownTransform) {
  // Create a CAD model cloud (cube)
  auto cad_cloud = makeCubeCloud(0.1, 10);  // 10cm cube, 600 points

  // Apply a known transform to create a "scene" cloud
  Eigen::Isometry3d known_transform = Eigen::Isometry3d::Identity();
  known_transform.translation() = Eigen::Vector3d(0.05, -0.02, 0.1);
  known_transform.linear() =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  auto scene_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*cad_cloud, *scene_cloud,
                           known_transform.matrix().cast<float>());

  // Use ICP with the known transform as initial guess (should converge)
  RegistrationConfig config;
  config.voxel_size = 0.003;
  config.icp_max_iterations = 50;
  HybridRegistrator registrator(config);

  auto result = registrator.refineIcp(cad_cloud, scene_cloud, known_transform);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.icp_fitness, 0.001);

  // Check transform accuracy
  Eigen::Vector3d trans_error =
      result.transform.translation() - known_transform.translation();
  EXPECT_LT(trans_error.norm(), 0.002);  // < 2mm translation error
}

TEST(HybridRegistrator, IcpRefineWithSmallPerturbation) {
  auto cad_cloud = makeCubeCloud(0.1, 10);

  Eigen::Isometry3d known_transform = Eigen::Isometry3d::Identity();
  known_transform.translation() = Eigen::Vector3d(0.1, 0.0, 0.3);
  known_transform.linear() =
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()).toRotationMatrix();

  auto scene_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*cad_cloud, *scene_cloud,
                           known_transform.matrix().cast<float>());

  // Perturb the initial guess slightly (simulating frame-to-frame tracking)
  Eigen::Isometry3d perturbed = known_transform;
  perturbed.translation() += Eigen::Vector3d(0.005, -0.003, 0.002);

  RegistrationConfig config;
  config.voxel_size = 0.003;
  HybridRegistrator registrator(config);

  auto result = registrator.refineIcp(cad_cloud, scene_cloud, perturbed);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.icp_fitness, 0.001);
}

}  // namespace
}  // namespace perspective_grasp
