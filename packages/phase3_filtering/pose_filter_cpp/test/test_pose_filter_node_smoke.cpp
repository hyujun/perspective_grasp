#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <perception_msgs/msg/associated_pose.hpp>
#include <perception_msgs/msg/associated_pose_array.hpp>
#include <perception_msgs/msg/pose_covariance_array.hpp>
#include <perception_msgs/msg/pose_with_meta_array.hpp>

#include "pose_filter_cpp/pose_filter_node.hpp"

using namespace std::chrono_literals;

namespace {

class PoseFilterSmokeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

rclcpp::NodeOptions makeOptions(bool use_associated_input,
                                double publish_rate_hz = 30.0) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      {"use_associated_input", use_associated_input},
      {"publish_rate_hz", publish_rate_hz},
      {"stale_timeout_sec", 2.0},
      {"output_frame_id", std::string("camera_color_optical_frame")},
      {"process_noise_pos", 0.001},
      {"process_noise_rot", 0.01},
      {"process_noise_vel", 0.1},
      {"iekf_max_iterations", 3},
      {"mahalanobis_threshold", 16.81},
  });
  return opts;
}

}  // namespace

// --------------------------------------------------------------------------
// Construction
// --------------------------------------------------------------------------

TEST_F(PoseFilterSmokeTest, ConstructsInAssociatedMode) {
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(
      makeOptions(/*use_associated_input=*/true));
  EXPECT_STREQ(node->get_name(), "pose_filter");
  EXPECT_TRUE(node->get_parameter("use_associated_input").as_bool());
}

TEST_F(PoseFilterSmokeTest, ConstructsInLegacyMode) {
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(
      makeOptions(/*use_associated_input=*/false));
  EXPECT_STREQ(node->get_name(), "pose_filter");
  EXPECT_FALSE(node->get_parameter("use_associated_input").as_bool());
}

TEST_F(PoseFilterSmokeTest, ExposedNoiseParametersAreReadable) {
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(
      makeOptions(true));
  // Every process + measurement noise parameter should be declared.
  EXPECT_DOUBLE_EQ(node->get_parameter("process_noise_pos").as_double(), 0.001);
  EXPECT_DOUBLE_EQ(node->get_parameter("process_noise_rot").as_double(), 0.01);
  EXPECT_DOUBLE_EQ(node->get_parameter("process_noise_omega").as_double(), 0.1);
  EXPECT_DOUBLE_EQ(node->get_parameter("process_noise_vel").as_double(), 0.1);
  EXPECT_DOUBLE_EQ(node->get_parameter("meas_noise_pos").as_double(), 0.005);
  EXPECT_DOUBLE_EQ(node->get_parameter("meas_noise_rot").as_double(), 0.05);
}

TEST_F(PoseFilterSmokeTest, FitnessMappingParametersAreReadable) {
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(
      makeOptions(true));
  EXPECT_DOUBLE_EQ(node->get_parameter("fitness_reference").as_double(),
                   0.001);
  EXPECT_DOUBLE_EQ(node->get_parameter("noise_scale_min").as_double(), 0.1);
  EXPECT_DOUBLE_EQ(node->get_parameter("noise_scale_max").as_double(), 10.0);
  EXPECT_DOUBLE_EQ(
      node->get_parameter("stale_predict_threshold_sec").as_double(), 1.0);
}

TEST_F(PoseFilterSmokeTest, OutputFrameParameterIsRead) {
  rclcpp::NodeOptions opts = makeOptions(true);
  opts.parameter_overrides({
      {"use_associated_input", true},
      {"output_frame_id", std::string("ur5e_base_link")},
  });
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(opts);
  EXPECT_EQ(node->get_parameter("output_frame_id").as_string(),
            "ur5e_base_link");
}

// --------------------------------------------------------------------------
// Topic interface
// --------------------------------------------------------------------------

TEST_F(PoseFilterSmokeTest, AdvertisesFilteredPosesTopic) {
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(
      makeOptions(true));
  auto listener = std::make_shared<rclcpp::Node>("filtered_listener");
  auto sub =
      listener->create_subscription<perception_msgs::msg::PoseWithMetaArray>(
          "/pose_filter/filtered_poses", rclcpp::QoS(1).best_effort(),
          [](perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr) {});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + 200ms;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_GE(sub->get_publisher_count(), 1u);
}

TEST_F(PoseFilterSmokeTest, AdvertisesCovarianceTopic) {
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(
      makeOptions(true));
  auto listener = std::make_shared<rclcpp::Node>("cov_listener");
  auto sub =
      listener->create_subscription<perception_msgs::msg::PoseCovarianceArray>(
          "/pose_filter/covariance", rclcpp::QoS(1).best_effort(),
          [](perception_msgs::msg::PoseCovarianceArray::ConstSharedPtr) {});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + 200ms;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_GE(sub->get_publisher_count(), 1u);
}

TEST_F(PoseFilterSmokeTest, PublishesDiagnosticsEvenWithoutInput) {
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(
      makeOptions(/*use_associated_input=*/true, /*publish_rate_hz=*/100.0));
  auto listener = std::make_shared<rclcpp::Node>("diag_listener");
  int received = 0;
  uint8_t last_level = 255;
  auto sub =
      listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/pose_filter/diagnostics", rclcpp::QoS(1).best_effort(),
          [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
            if (!msg->status.empty()) {
              ++received;
              last_level = msg->status.front().level;
            }
          });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(listener);

  const auto deadline = std::chrono::steady_clock::now() + 400ms;
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_GT(received, 0);
  // With no input, we expect WARN (no active filters).
  EXPECT_EQ(last_level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

// --------------------------------------------------------------------------
// End-to-end: publish an associated pose, expect a filtered pose out
// --------------------------------------------------------------------------

TEST_F(PoseFilterSmokeTest, AssociatedInputProducesFilteredOutput) {
  auto node = std::make_shared<perspective_grasp::PoseFilterNode>(
      makeOptions(/*use_associated_input=*/true, /*publish_rate_hz=*/100.0));
  auto talker = std::make_shared<rclcpp::Node>("associator_stub");
  auto pub = talker->create_publisher<perception_msgs::msg::AssociatedPoseArray>(
      "/associated/poses", rclcpp::QoS(1).best_effort());

  auto listener = std::make_shared<rclcpp::Node>("filtered_listener");
  int received = 0;
  auto sub =
      listener->create_subscription<perception_msgs::msg::PoseWithMetaArray>(
          "/pose_filter/filtered_poses", rclcpp::QoS(1).best_effort(),
          [&](perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr msg) {
            if (!msg->poses.empty()) ++received;
          });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(talker);
  exec.add_node(listener);

  // Spin to let discovery + QoS matching happen.
  const auto warmup = std::chrono::steady_clock::now() + 150ms;
  while (std::chrono::steady_clock::now() < warmup) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  // Publish a few poses over a short window so the filter initializes.
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    perception_msgs::msg::AssociatedPoseArray msg;
    msg.header.stamp = talker->now();
    msg.header.frame_id = "camera_color_optical_frame";
    perception_msgs::msg::AssociatedPose ap;
    ap.global_id = 42;
    ap.class_name = "cup";
    ap.num_observing_cameras = 1;
    ap.pose.header = msg.header;
    ap.pose.pose.position.x = 0.3;
    ap.pose.pose.position.y = 0.1;
    ap.pose.pose.position.z = 0.5;
    ap.pose.pose.orientation.w = 1.0;
    msg.poses.push_back(ap);
    pub->publish(msg);

    exec.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  EXPECT_GT(received, 0) << "pose_filter never republished a filtered pose";
}
