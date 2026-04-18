#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <set>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <perception_msgs/msg/pose_with_meta_array.hpp>

#include "cross_camera_associator/camera_pose_buffer.hpp"

using perspective_grasp::CameraPoseBuffer;
using perspective_grasp::TimedDetections;

namespace {

perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr makeMsg(
    double stamp_sec) {
  auto msg = std::make_shared<perception_msgs::msg::PoseWithMetaArray>();
  msg->header.stamp.sec = static_cast<int32_t>(stamp_sec);
  msg->header.stamp.nanosec =
      static_cast<uint32_t>((stamp_sec - static_cast<int32_t>(stamp_sec)) * 1e9);
  return msg;
}

rclcpp::Time makeTime(double sec) {
  // Match the clock type used by `rclcpp::Time(msg->header.stamp)` inside
  // CameraPoseBuffer::update (RCL_ROS_TIME); otherwise subtraction throws.
  return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
}

class CameraPoseBufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

}  // namespace

TEST_F(CameraPoseBufferTest, EmptyBufferReturnsEmptySnapshot) {
  CameraPoseBuffer buf;
  auto snap = buf.snapshot(makeTime(1.0), 100.0);
  EXPECT_TRUE(snap.empty());
}

TEST_F(CameraPoseBufferTest, UpdateStoresAndSnapshotReturns) {
  CameraPoseBuffer buf;
  buf.update(0, makeMsg(1.0));

  auto snap = buf.snapshot(makeTime(1.01), 100.0);  // 10ms age
  ASSERT_EQ(snap.size(), 1u);
  EXPECT_EQ(snap[0].camera_id, 0);
}

TEST_F(CameraPoseBufferTest, SecondUpdateOverwritesSameCameraId) {
  CameraPoseBuffer buf;
  buf.update(0, makeMsg(1.0));
  buf.update(0, makeMsg(2.0));

  auto snap = buf.snapshot(makeTime(2.01), 100.0);
  ASSERT_EQ(snap.size(), 1u);
  // Timestamp reflects the newer message.
  EXPECT_NEAR(snap[0].stamp.seconds(), 2.0, 1e-6);
}

TEST_F(CameraPoseBufferTest, SnapshotFiltersByAgeMs) {
  CameraPoseBuffer buf;
  buf.update(0, makeMsg(1.0));
  buf.update(1, makeMsg(1.5));

  // At t=1.55, max_age=100ms → cam1 age=50ms OK, cam0 age=550ms stale.
  auto snap = buf.snapshot(makeTime(1.55), 100.0);
  ASSERT_EQ(snap.size(), 1u);
  EXPECT_EQ(snap[0].camera_id, 1);
}

TEST_F(CameraPoseBufferTest, SnapshotReturnsAllCamerasWithinAge) {
  CameraPoseBuffer buf;
  buf.update(0, makeMsg(1.00));
  buf.update(1, makeMsg(1.01));
  buf.update(2, makeMsg(1.02));

  auto snap = buf.snapshot(makeTime(1.05), 100.0);
  ASSERT_EQ(snap.size(), 3u);

  std::set<int> cam_ids;
  for (const auto& td : snap) {
    cam_ids.insert(td.camera_id);
  }
  EXPECT_EQ(cam_ids, (std::set<int>{0, 1, 2}));
}

TEST_F(CameraPoseBufferTest, NegativeAgeFutureMessageIsExcluded) {
  // Messages timestamped after 'now' are skipped (guards against clock skew).
  CameraPoseBuffer buf;
  buf.update(0, makeMsg(5.0));

  auto snap = buf.snapshot(makeTime(1.0), 100.0);
  EXPECT_TRUE(snap.empty());
}

TEST_F(CameraPoseBufferTest, ConcurrentUpdatesAreSafe) {
  // Smoke-test the internal mutex: multiple writers + a reader should not
  // crash or deadlock. Does not assert strict ordering.
  CameraPoseBuffer buf;
  std::atomic<bool> stop{false};

  std::thread writer_a([&] {
    double t = 0.0;
    while (!stop.load()) {
      buf.update(0, makeMsg(t));
      t += 0.001;
    }
  });
  std::thread writer_b([&] {
    double t = 0.0;
    while (!stop.load()) {
      buf.update(1, makeMsg(t));
      t += 0.001;
    }
  });

  for (int i = 0; i < 200; ++i) {
    buf.snapshot(makeTime(10.0), 1e6);  // wide age to accept any stamp
  }

  stop.store(true);
  writer_a.join();
  writer_b.join();

  SUCCEED();
}
