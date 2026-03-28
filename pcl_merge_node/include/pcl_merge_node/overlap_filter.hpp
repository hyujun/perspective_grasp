#pragma once

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perspective_grasp {

class OverlapFilter {
public:
  explicit OverlapFilter(double overlap_radius = 0.005);

  /// Score each point in merged by counting how many source clouds
  /// (cloud_a, cloud_b) have a neighbor within overlap_radius.
  void process(pcl::PointCloud<pcl::PointXYZ>::Ptr& merged,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_a,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_b);

  /// Return the overlap count for a given point index.
  /// 1 = seen by one camera only, 2 = seen by both cameras.
  int getOverlapCount(size_t idx) const;

  /// Return the full overlap count vector.
  const std::vector<int>& getOverlapCounts() const;

  void setOverlapRadius(double radius);

private:
  double overlap_radius_;
  std::vector<int> overlap_counts_;
};

}  // namespace perspective_grasp
