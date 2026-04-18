#include <gtest/gtest.h>

#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "teaser_icp_hybrid/fpfh_feature_extractor.hpp"
#include "test_helpers.hpp"

namespace perspective_grasp {
namespace {

using test_helpers::makeCubeCloud;

TEST(FpfhFeatureExtractor, ReturnsOneDescriptorPerPoint) {
  auto cloud = makeCubeCloud(0.1, 10);
  FpfhFeatureExtractor extractor(0.015, 0.025);

  auto features = extractor.extract(cloud);
  ASSERT_NE(features, nullptr);
  EXPECT_EQ(features->size(), cloud->size());
}

TEST(FpfhFeatureExtractor, DescriptorsAreFinite) {
  auto cloud = makeCubeCloud(0.1, 10);
  FpfhFeatureExtractor extractor(0.015, 0.025);

  auto features = extractor.extract(cloud);
  ASSERT_NE(features, nullptr);
  ASSERT_GT(features->size(), 0u);

  for (const auto& f : *features) {
    for (int i = 0; i < 33; ++i) {
      EXPECT_TRUE(std::isfinite(f.histogram[i]))
          << "FPFH histogram bin " << i << " should be finite";
    }
  }
}

TEST(FpfhFeatureExtractor, HandlesSparseCloudWithoutCrash) {
  // Too few points for radius search — should not crash (may produce NaN
  // histograms, which is acceptable as long as the call returns).
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->emplace_back(0.0f, 0.0f, 0.0f);
  cloud->emplace_back(0.001f, 0.0f, 0.0f);
  FpfhFeatureExtractor extractor(0.01, 0.02);
  auto features = extractor.extract(cloud);
  ASSERT_NE(features, nullptr);
  EXPECT_EQ(features->size(), cloud->size());
}

}  // namespace
}  // namespace perspective_grasp
