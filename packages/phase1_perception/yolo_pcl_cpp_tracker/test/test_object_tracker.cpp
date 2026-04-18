#include <gtest/gtest.h>

#include "yolo_pcl_cpp_tracker/object_tracker.hpp"

namespace perspective_grasp {
namespace {

TEST(ObjectTracker, DefaultStateNeedsGlobalReInit) {
  ObjectTracker t;
  EXPECT_FALSE(t.initialized);
  EXPECT_TRUE(t.needsGlobalReInit())
      << "fresh tracker should request global re-init";
  EXPECT_FALSE(t.isStale());
}

TEST(ObjectTracker, MarkTrackedPopulatesState) {
  ObjectTracker t;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);

  t.markTracked(pose, 1e-5);

  EXPECT_TRUE(t.initialized);
  EXPECT_EQ(t.lost_count, 0);
  EXPECT_DOUBLE_EQ(t.last_fitness, 1e-5);
  EXPECT_TRUE(t.last_pose.isApprox(pose));
  EXPECT_FALSE(t.needsGlobalReInit())
      << "good fitness + initialized should not trigger re-init";
}

TEST(ObjectTracker, HighFitnessForcesReInit) {
  ObjectTracker t;
  t.markTracked(Eigen::Isometry3d::Identity(),
                ObjectTracker::kFitnessThreshold * 2.0);
  EXPECT_TRUE(t.needsGlobalReInit())
      << "fitness above threshold should force re-init";
}

TEST(ObjectTracker, LostCountIncrementsAndTriggersReInit) {
  ObjectTracker t;
  t.markTracked(Eigen::Isometry3d::Identity(), 1e-5);

  for (int i = 0; i <= ObjectTracker::kMaxLostFrames; ++i) {
    t.markLost();
  }
  EXPECT_EQ(t.lost_count, ObjectTracker::kMaxLostFrames + 1);
  EXPECT_TRUE(t.needsGlobalReInit());
  EXPECT_FALSE(t.isStale())
      << "just past re-init threshold is not yet stale (3x threshold)";
}

TEST(ObjectTracker, StaleAfterThreeTimesMaxLost) {
  ObjectTracker t;
  t.markTracked(Eigen::Isometry3d::Identity(), 1e-5);

  for (int i = 0; i <= ObjectTracker::kMaxLostFrames * 3; ++i) {
    t.markLost();
  }
  EXPECT_TRUE(t.isStale());
}

TEST(ObjectTracker, MarkTrackedResetsLostCount) {
  ObjectTracker t;
  for (int i = 0; i < 5; ++i) t.markLost();
  EXPECT_EQ(t.lost_count, 5);

  t.markTracked(Eigen::Isometry3d::Identity(), 1e-5);
  EXPECT_EQ(t.lost_count, 0);
}

}  // namespace
}  // namespace perspective_grasp
