#include "pcl_merge_node/overlap_filter.hpp"

#include <pcl/kdtree/kdtree_flann.h>

namespace perspective_grasp {

OverlapFilter::OverlapFilter(double overlap_radius)
    : overlap_radius_(overlap_radius) {}

void OverlapFilter::process(pcl::PointCloud<pcl::PointXYZ>::Ptr& merged,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_a,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_b) {
  overlap_counts_.clear();

  if (!merged || merged->empty()) {
    return;
  }

  overlap_counts_.resize(merged->size(), 0);

  // Build KD-trees for each source cloud
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_a;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_b;

  bool a_valid = cloud_a && !cloud_a->empty();
  bool b_valid = cloud_b && !cloud_b->empty();

  if (a_valid) {
    tree_a.setInputCloud(cloud_a);
  }
  if (b_valid) {
    tree_b.setInputCloud(cloud_b);
  }

  std::vector<int> indices;
  std::vector<float> distances;

  for (size_t i = 0; i < merged->size(); ++i) {
    int count = 0;
    const auto& pt = (*merged)[i];

    if (a_valid) {
      indices.clear();
      distances.clear();
      if (tree_a.radiusSearch(pt, overlap_radius_, indices, distances, 1) > 0) {
        ++count;
      }
    }

    if (b_valid) {
      indices.clear();
      distances.clear();
      if (tree_b.radiusSearch(pt, overlap_radius_, indices, distances, 1) > 0) {
        ++count;
      }
    }

    overlap_counts_[i] = count;
  }
}

int OverlapFilter::getOverlapCount(size_t idx) const {
  if (idx < overlap_counts_.size()) {
    return overlap_counts_[idx];
  }
  return 0;
}

const std::vector<int>& OverlapFilter::getOverlapCounts() const {
  return overlap_counts_;
}

void OverlapFilter::setOverlapRadius(double radius) {
  overlap_radius_ = radius;
}

}  // namespace perspective_grasp
