#include "teaser_icp_hybrid/hybrid_registrator.hpp"

#include <chrono>

#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#ifdef HAS_TEASERPP
#include <teaser/registration.h>
#endif

namespace perspective_grasp {

HybridRegistrator::HybridRegistrator(const RegistrationConfig& config)
    : config_(config),
      feature_extractor_(std::make_unique<FpfhFeatureExtractor>(
          config.fpfh_normal_radius, config.fpfh_feature_radius)) {}

void HybridRegistrator::updateConfig(const RegistrationConfig& config) {
  config_ = config;
  feature_extractor_ = std::make_unique<FpfhFeatureExtractor>(
      config.fpfh_normal_radius, config.fpfh_feature_radius);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr HybridRegistrator::downsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  float leaf = static_cast<float>(config_.voxel_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*filtered);
  return filtered;
}

RegistrationResult HybridRegistrator::align(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) const {
  RegistrationResult result;
  auto t_start = std::chrono::steady_clock::now();

  // Step 1: Downsample both clouds
  auto src_down = downsample(source);
  auto tgt_down = downsample(target);

  if (src_down->empty() || tgt_down->empty()) {
    return result;
  }

#ifdef HAS_TEASERPP
  // Step 2: Extract FPFH features
  auto src_features = feature_extractor_->extract(src_down);
  auto tgt_features = feature_extractor_->extract(tgt_down);

  // Step 3: Find correspondences via nearest neighbor in feature space
  pcl::search::KdTree<pcl::FPFHSignature33> feature_tree;
  feature_tree.setInputCloud(tgt_features);

  const size_t n_src = src_down->size();
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_matched(3, static_cast<Eigen::Index>(n_src));
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_matched(3, static_cast<Eigen::Index>(n_src));

  size_t valid_count = 0;
  for (size_t i = 0; i < n_src; ++i) {
    pcl::Indices indices(1);
    std::vector<float> distances(1);
    if (feature_tree.nearestKSearch((*src_features)[i], 1, indices, distances) > 0) {
      src_matched.col(static_cast<Eigen::Index>(valid_count)) =
          (*src_down)[i].getVector3fMap().cast<double>();
      tgt_matched.col(static_cast<Eigen::Index>(valid_count)) =
          (*tgt_down)[indices[0]].getVector3fMap().cast<double>();
      ++valid_count;
    }
  }

  if (valid_count < 3) {
    return result;
  }

  src_matched.conservativeResize(3, static_cast<Eigen::Index>(valid_count));
  tgt_matched.conservativeResize(3, static_cast<Eigen::Index>(valid_count));

  // Step 4: TEASER++ global registration
  teaser::RobustRegistrationSolver::Params teaser_params;
  teaser_params.noise_bound = config_.noise_bound;
  teaser_params.cbar2 = config_.cbar2;
  teaser_params.estimate_scaling = config_.estimate_scaling;
  teaser_params.rotation_max_iterations = config_.rotation_max_iterations;
  teaser_params.rotation_cost_threshold = config_.rotation_cost_threshold;
  teaser_params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;

  teaser::RobustRegistrationSolver solver(teaser_params);
  solver.solve(src_matched, tgt_matched);

  auto teaser_solution = solver.getSolution();

  Eigen::Isometry3d teaser_transform = Eigen::Isometry3d::Identity();
  teaser_transform.linear() = teaser_solution.rotation;
  teaser_transform.translation() = teaser_solution.translation;

  // Step 5: ICP local refinement using TEASER++ result as initial guess
  auto icp_result = refineIcp(source, target, teaser_transform);
#else
  // Without TEASER++, fall back to ICP with identity initial guess
  auto icp_result = refineIcp(source, target, Eigen::Isometry3d::Identity());
#endif

  auto t_end = std::chrono::steady_clock::now();

  result.transform = icp_result.transform;
  result.teaser_fitness = 0.0;
  result.icp_fitness = icp_result.icp_fitness;
  result.success = icp_result.success;
  result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  return result;
}

RegistrationResult HybridRegistrator::refineIcp(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Eigen::Isometry3d& initial_guess) const {
  RegistrationResult result;
  auto t_start = std::chrono::steady_clock::now();

  auto src_down = downsample(source);
  auto tgt_down = downsample(target);

  if (src_down->empty() || tgt_down->empty()) {
    return result;
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src_down);
  icp.setInputTarget(tgt_down);
  icp.setMaximumIterations(config_.icp_max_iterations);
  icp.setTransformationEpsilon(config_.icp_transformation_epsilon);
  icp.setEuclideanFitnessEpsilon(config_.icp_euclidean_fitness_epsilon);
  icp.setMaxCorrespondenceDistance(config_.icp_max_correspondence_distance);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned, initial_guess.matrix().cast<float>());

  auto t_end = std::chrono::steady_clock::now();

  result.success = icp.hasConverged();
  result.icp_fitness = icp.getFitnessScore();
  result.transform = Eigen::Isometry3d(
      icp.getFinalTransformation().cast<double>());
  result.elapsed_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();

  return result;
}

}  // namespace perspective_grasp
