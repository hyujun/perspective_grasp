#include "teaser_icp_hybrid/fpfh_feature_extractor.hpp"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

namespace perspective_grasp {

FpfhFeatureExtractor::FpfhFeatureExtractor(
    double normal_radius, double feature_radius)
    : normal_radius_(normal_radius), feature_radius_(feature_radius) {}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr FpfhFeatureExtractor::extract(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
  // Estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_est;
  normal_est.setInputCloud(cloud);
  normal_est.setRadiusSearch(normal_radius_);
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  normal_est.setSearchMethod(tree);
  normal_est.compute(*normals);

  // Compute FPFH features
  auto fpfh_features =
      std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
      fpfh_est;
  fpfh_est.setInputCloud(cloud);
  fpfh_est.setInputNormals(normals);
  fpfh_est.setRadiusSearch(feature_radius_);
  fpfh_est.setSearchMethod(tree);
  fpfh_est.compute(*fpfh_features);

  return fpfh_features;
}

}  // namespace perspective_grasp
