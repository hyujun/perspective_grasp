#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "teaser_icp_hybrid/hybrid_registrator.hpp"
#include "test_helpers.hpp"

namespace perspective_grasp {
namespace {

using test_helpers::addGaussianNoise;
using test_helpers::makeCubeCloud;

// ---------------- refineIcp (always available) ----------------

TEST(HybridRegistrator, IcpRefineKnownTransform) {
  auto cad_cloud = makeCubeCloud(0.1, 10);

  Eigen::Isometry3d known_transform = Eigen::Isometry3d::Identity();
  known_transform.translation() = Eigen::Vector3d(0.05, -0.02, 0.1);
  known_transform.linear() =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  auto scene_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*cad_cloud, *scene_cloud,
                           known_transform.matrix().cast<float>());

  RegistrationConfig config;
  config.voxel_size = 0.003;
  config.icp_max_iterations = 50;
  HybridRegistrator registrator(config);

  auto result = registrator.refineIcp(cad_cloud, scene_cloud, known_transform);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.icp_fitness, 0.001);
  EXPECT_LT((result.transform.translation() - known_transform.translation())
                .norm(),
            0.002);
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

  Eigen::Isometry3d perturbed = known_transform;
  perturbed.translation() += Eigen::Vector3d(0.005, -0.003, 0.002);

  RegistrationConfig config;
  config.voxel_size = 0.003;
  HybridRegistrator registrator(config);

  auto result = registrator.refineIcp(cad_cloud, scene_cloud, perturbed);
  EXPECT_TRUE(result.success);
  EXPECT_LT(result.icp_fitness, 0.001);
}

TEST(HybridRegistrator, IcpRefineWithNoisyScene) {
  auto cad_cloud = makeCubeCloud(0.1, 10);

  Eigen::Isometry3d known_transform = Eigen::Isometry3d::Identity();
  known_transform.translation() = Eigen::Vector3d(0.02, 0.01, 0.2);

  auto scene_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*cad_cloud, *scene_cloud,
                           known_transform.matrix().cast<float>());
  addGaussianNoise(*scene_cloud, 0.002, /*seed=*/7);  // 2mm noise

  RegistrationConfig config;
  config.voxel_size = 0.003;
  HybridRegistrator registrator(config);

  auto result = registrator.refineIcp(cad_cloud, scene_cloud, known_transform);
  EXPECT_TRUE(result.success);
  EXPECT_LT((result.transform.translation() - known_transform.translation())
                .norm(),
            0.005)
      << "ICP should still recover within 5mm under 2mm point noise";
}

TEST(HybridRegistrator, EmptyInputsReportFailure) {
  auto empty = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto cube = makeCubeCloud(0.1, 10);

  HybridRegistrator registrator;
  auto r1 = registrator.refineIcp(empty, cube, Eigen::Isometry3d::Identity());
  EXPECT_FALSE(r1.success);
  auto r2 = registrator.refineIcp(cube, empty, Eigen::Isometry3d::Identity());
  EXPECT_FALSE(r2.success);
}

TEST(HybridRegistrator, ElapsedMsIsPositive) {
  auto cad = makeCubeCloud(0.1, 8);
  auto scene = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cad);

  HybridRegistrator registrator;
  auto r = registrator.refineIcp(cad, scene, Eigen::Isometry3d::Identity());
  EXPECT_GT(r.elapsed_ms, 0.0);
}

// ---------------- updateConfig ----------------

TEST(HybridRegistrator, UpdateConfigAffectsNextCall) {
  auto cad = makeCubeCloud(0.1, 10);
  auto scene = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cad);

  RegistrationConfig tight;
  tight.voxel_size = 0.003;
  tight.icp_max_iterations = 50;
  HybridRegistrator registrator(tight);
  auto good = registrator.refineIcp(cad, scene, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(good.success);

  // Reconfigure with 0 iterations — ICP should not improve fit, so refined
  // transform is effectively the initial guess (identity).
  RegistrationConfig no_iters = tight;
  no_iters.icp_max_iterations = 0;
  registrator.updateConfig(no_iters);
  auto pinned =
      registrator.refineIcp(cad, scene, Eigen::Isometry3d::Identity());
  EXPECT_TRUE(pinned.transform.isApprox(Eigen::Isometry3d::Identity(), 1e-6))
      << "0 ICP iterations should leave transform at the initial guess";
}

// ---------------- align (TEASER++ only) ----------------

#ifdef HAS_TEASERPP
TEST(HybridRegistrator, AlignRecoversKnownTransform) {
  auto cad = makeCubeCloud(0.1, 12);

  Eigen::Isometry3d known = Eigen::Isometry3d::Identity();
  known.translation() = Eigen::Vector3d(0.08, -0.04, 0.15);
  known.linear() =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  auto scene = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::transformPointCloud(*cad, *scene, known.matrix().cast<float>());

  RegistrationConfig config;
  config.voxel_size = 0.005;
  config.fpfh_normal_radius = 0.015;
  config.fpfh_feature_radius = 0.03;
  config.noise_bound = 0.01;
  HybridRegistrator registrator(config);

  auto result = registrator.align(cad, scene);
  EXPECT_TRUE(result.success);
  EXPECT_LT((result.transform.translation() - known.translation()).norm(),
            0.01)
      << "global align + ICP should recover translation within 1cm";
}

TEST(HybridRegistrator, AlignEmptyInputFails) {
  auto cad = makeCubeCloud(0.1, 10);
  auto empty = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  HybridRegistrator registrator;
  auto r = registrator.align(cad, empty);
  EXPECT_FALSE(r.success);
}
#else
TEST(HybridRegistrator, AlignTestsSkippedNoTeaser) {
  GTEST_SKIP() << "TEASER++ not available — align() tests are not built";
}
#endif

}  // namespace
}  // namespace perspective_grasp
