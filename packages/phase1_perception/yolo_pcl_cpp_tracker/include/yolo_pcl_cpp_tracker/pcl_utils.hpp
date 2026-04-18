#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/region_of_interest.hpp>

namespace perspective_grasp::pcl_utils {

/// Crop an organized PCL cloud to the 2D bounding box defined by `roi`.
/// Only finite points with positive depth are kept. For unorganized input
/// the returned cloud is empty, matching the original node behavior.
pcl::PointCloud<pcl::PointXYZ>::Ptr cropToRoi(
    const pcl::PointCloud<pcl::PointXYZ>& full_cloud,
    const sensor_msgs::msg::RegionOfInterest& roi);

/// Voxel downsample then statistical outlier removal. Returns the filtered
/// cloud; if the voxel step empties it, the statistical step is skipped and
/// the empty cloud is returned as-is.
pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double voxel_leaf_size,
    int outlier_mean_k,
    double outlier_stddev);

}  // namespace perspective_grasp::pcl_utils
