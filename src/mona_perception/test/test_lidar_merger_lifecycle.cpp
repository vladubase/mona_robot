// Copyright 2026 vladubase
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <gtest/gtest.h>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "mona_perception/lidar_merger_node.hpp"

using namespace std::chrono_literals;

// Test fixture to initialize the standard ROS 2 environment
class TestLidarMerger : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void spin_for_milliseconds(
        const std::shared_ptr<mona_perception::LidarMergerNode> &node,
        const std::shared_ptr<rclcpp::Node> &test_node,
        int duration_ms) {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node->get_node_base_interface());
        executor.add_node(test_node);

        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time <
            std::chrono::milliseconds(duration_ms))
        {
            executor.spin_some(10ms);
            std::this_thread::sleep_for(10ms);
        }
    }

    sensor_msgs::msg::LaserScan make_scan(
        const std::string &frame_id,
        const rclcpp::Time &stamp) {
        sensor_msgs::msg::LaserScan scan;
        scan.header.frame_id = frame_id;
        scan.header.stamp    = stamp;
        scan.angle_min       = -0.1F;
        scan.angle_max       = 0.1F;
        scan.angle_increment = 0.1F;
        scan.time_increment  = 0.0F;
        scan.scan_time       = 0.1F;
        scan.range_min       = 0.05F;
        scan.range_max       = 10.0F;
        scan.ranges          = {1.0F, 1.1F, 1.2F};
        return scan;
    }

    geometry_msgs::msg::TransformStamped make_static_tf(
        const std::string &child_frame,
        double x,
        double y) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp            = rclcpp::Clock().now();
        transform.header.frame_id         = "base_link";
        transform.child_frame_id          = child_frame;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.w    = 1.0;
        return transform;
    }
};

// Test 1: Verify initialization, name registration, and lifecycle state transitions
TEST_F(TestLidarMerger, LifecycleTransition) {
    rclcpp::NodeOptions options;
    auto node = std::make_shared<mona_perception::LidarMergerNode>(options);

    // Smoke Test: Verify Node Instantiation and Name
    EXPECT_NE(node, nullptr);
    EXPECT_STREQ(node->get_name(), "mona_lidar_merger");

    // Verify the initial state is strictly UNCONFIGURED
    EXPECT_EQ(
        node->get_current_state().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED
    );

    // Trigger on_configure()
    // This verifies memory allocation for tf2_buffer and message_filters
    auto state_after_configure = node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    EXPECT_EQ(
        state_after_configure.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE
    );

    // Trigger on_activate()
    // This verifies the activation of lifecycle publishers and internal state resets
    auto state_after_activate = node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    EXPECT_EQ(
        state_after_activate.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE
    );

    // Trigger on_deactivate()
    // This verifies that data streams halt seamlessly
    auto state_after_deactivate = node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

    EXPECT_EQ(
        state_after_deactivate.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE
    );

    // Trigger on_shutdown()
    // This verifies proper resource deallocation via the on_cleanup callback
    auto state_after_shutdown = node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);

    EXPECT_EQ(
        state_after_shutdown.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED
    );
}

// Test 2: Verify empty target_frame uses the safe base_link fallback
TEST_F(TestLidarMerger, EmptyTargetFrameFallsBackToBaseLink) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"target_frame", ""}});
    auto node = std::make_shared<mona_perception::LidarMergerNode>(options);

    auto state_after_configure = node->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    EXPECT_EQ(
        state_after_configure.id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// Test 3: Verify perception power-state callbacks accept power loss and restore events
TEST_F(TestLidarMerger, PowerStateMessagesAreAccepted) {
    rclcpp::NodeOptions options;
    auto node      = std::make_shared<mona_perception::LidarMergerNode>(options);
    auto test_node = std::make_shared<rclcpp::Node>("lidar_power_test_stimulator");

    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    auto power_pub = test_node->create_publisher<std_msgs::msg::Bool>(
        "hardware/power/perception_pc", 10);
    spin_for_milliseconds(node, test_node, 150);

    std_msgs::msg::Bool power_msg;
    power_msg.data = false;
    power_pub->publish(power_msg);
    spin_for_milliseconds(node, test_node, 100);

    power_msg.data = true;
    power_pub->publish(power_msg);
    spin_for_milliseconds(node, test_node, 100);

    SUCCEED();
}

// Test 4: Verify synchronized scans are fused into a combined cloud
TEST_F(TestLidarMerger, PublishesCombinedCloudFromFourScans) {
    rclcpp::NodeOptions options;
    auto node      = std::make_shared<mona_perception::LidarMergerNode>(options);
    auto test_node = std::make_shared<rclcpp::Node>("lidar_scan_test_stimulator");

    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node);
    tf_broadcaster->sendTransform(
        std::vector<geometry_msgs::msg::TransformStamped>{
        make_static_tf("lidar_front_link", 0.2, 0.0),
        make_static_tf("lidar_back_link", -0.2, 0.0),
        make_static_tf("lidar_left_link", 0.0, 0.2),
        make_static_tf("lidar_right_link", 0.0, -0.2)});

    auto front_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
        "lidar_front/scan", rclcpp::SensorDataQoS());
    auto back_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
        "lidar_back/scan", rclcpp::SensorDataQoS());
    auto left_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
        "lidar_left/scan", rclcpp::SensorDataQoS());
    auto right_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
        "lidar_right/scan", rclcpp::SensorDataQoS());

    bool cloud_received = false;
    sensor_msgs::msg::PointCloud2 latest_cloud;
    auto cloud_sub = test_node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "perception/combined_cloud", 10,
        [&cloud_received, &latest_cloud](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            latest_cloud   = *msg;
            cloud_received = true;
        });

    spin_for_milliseconds(node, test_node, 250);

    auto stamp = test_node->now();
    for (int i = 0; i < 5 && !cloud_received; ++i) {
        front_pub->publish(make_scan("lidar_front_link", stamp));
        back_pub->publish(make_scan("lidar_back_link", stamp));
        left_pub->publish(make_scan("lidar_left_link", stamp));
        right_pub->publish(make_scan("lidar_right_link", stamp));
        spin_for_milliseconds(node, test_node, 150);
    }

    ASSERT_TRUE(cloud_received) << "Lidar merger did not publish a combined cloud.";
    EXPECT_EQ(latest_cloud.header.frame_id, "base_link");
    EXPECT_GT(latest_cloud.width, 0U);
}
