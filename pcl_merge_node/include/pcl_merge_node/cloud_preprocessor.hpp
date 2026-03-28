#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perspective_grasp {

struct PreprocessorParams {
  double table_height = 0.0;
  double max_object_height = 0.30;
  int outlier_mean_k = 30;
  double outlier_stddev = 1.5;
};

class CloudPreprocessor {
public:
  explicit CloudPreprocessor(const PreprocessorParams& params);

  /// Apply PassThrough (Z) + StatisticalOutlierRemoval to the input cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) const;

  void setParams(const PreprocessorParams& params);

private:
  PreprocessorParams params_;
};

}  // namespace perspective_grasp
