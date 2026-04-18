#include "yolo_pcl_cpp_tracker/pcl_utils.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace perspective_grasp::pcl_utils {

pcl::PointCloud<pcl::PointXYZ>::Ptr cropToRoi(
    const pcl::PointCloud<pcl::PointXYZ>& full_cloud,
    const sensor_msgs::msg::RegionOfInterest& roi) {
  auto cropped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (full_cloud.isOrganized()) {
    uint32_t x_start = roi.x_offset;
    uint32_t y_start = roi.y_offset;
    uint32_t x_end = std::min(x_start + roi.width, full_cloud.width);
    uint32_t y_end = std::min(y_start + roi.height, full_cloud.height);

    for (uint32_t y = y_start; y < y_end; ++y) {
      for (uint32_t x = x_start; x < x_end; ++x) {
        const auto& pt = full_cloud.at(x, y);
        if (std::isfinite(pt.x) && std::isfinite(pt.y) &&
            std::isfinite(pt.z) && pt.z > 0.0f) {
          cropped->push_back(pt);
        }
      }
    }
  }

  cropped->width = static_cast<uint32_t>(cropped->size());
  cropped->height = 1;
  cropped->is_dense = true;
  return cropped;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double voxel_leaf_size,
    int outlier_mean_k,
    double outlier_stddev) {
  auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  float leaf = static_cast<float>(voxel_leaf_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*downsampled);

  if (downsampled->empty()) {
    return downsampled;
  }

  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(downsampled);
  sor.setMeanK(outlier_mean_k);
  sor.setStddevMulThresh(outlier_stddev);
  sor.filter(*filtered);

  return filtered;
}

}  // namespace perspective_grasp::pcl_utils
