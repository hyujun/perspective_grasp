#include "pcl_merge_node/cloud_preprocessor.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace perspective_grasp {

CloudPreprocessor::CloudPreprocessor(const PreprocessorParams& params)
    : params_(params) {}

pcl::PointCloud<pcl::PointXYZ>::Ptr
CloudPreprocessor::process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) const {
  if (!input || input->empty()) {
    return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  // 1. PassThrough filter on Z axis
  auto after_passthrough = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(
      static_cast<float>(params_.table_height - 0.05),
      static_cast<float>(params_.table_height + params_.max_object_height));
  pass.filter(*after_passthrough);

  if (after_passthrough->empty()) {
    return after_passthrough;
  }

  // 2. Statistical Outlier Removal
  auto after_sor = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(after_passthrough);
  sor.setMeanK(params_.outlier_mean_k);
  sor.setStddevMulThresh(params_.outlier_stddev);
  sor.filter(*after_sor);

  return after_sor;
}

void CloudPreprocessor::setParams(const PreprocessorParams& params) {
  params_ = params;
}

}  // namespace perspective_grasp
