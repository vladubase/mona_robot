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
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "mona_control/twist_mux_node.hpp"
#include "mona_msgs/msg/fdir_state.hpp"
#include "mona_msgs/msg/twist_mux_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/**
 * @class TwistMuxSafetyFixture
 * @brief Test fixture for verifying hardware interlocks
 * and priority multiplexing within the TwistMuxNode.
 */
class TwistMuxSafetyFixture : public ::testing::Test {
protected:
    // --- Global Setup/Teardown for the Test Suite ---
    static void SetUpTestSuite() {
        setenv("ROS_DOMAIN_ID", "11", 1);
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    // --- Per-Test Setup/Teardown ---
    void SetUp() override {
        // 1. Initialize node with zero EMA alpha to disable
        // smoothing for strict equality assertions
        rclcpp::NodeOptions options;
        options.parameter_overrides(
        {
            {"command_timeout", 0.5},
            {"manual_takeover_time", 2.0},
            {"ema_alpha", 1.0}  // 1.0 = instant acceleration (no smoothing)
        });

        node_ = std::make_shared<mona_control::TwistMuxNode>(options);

        // 2. Transition node to ACTIVE state
        node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

        // 3. Create test publishers to simulate incoming FDIR and Teleop data
        test_node_ = std::make_shared<rclcpp::Node>("twist_mux_test_stimulator");
        fdir_pub_  = test_node_->create_publisher<mona_msgs::msg::FdirState>(
            "system/health_state", 10);
        teleop_pub_ = test_node_->create_publisher<geometry_msgs::msg::Twist>("cmd_teleop", 10);

        // 4. Create test subscribers to monitor output
        smooth_sub_ = test_node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_smoothed", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_cmd_   = *msg;
                cmd_received_ = true;
            });

        state_sub_ = test_node_->create_subscription<mona_msgs::msg::TwistMuxState>(
            "mux_state", 10,
            [this](const mona_msgs::msg::TwistMuxState::SharedPtr msg) {
                latest_state_ = msg->current_state;
            });

        // Initialize state
        cmd_received_ = false;
        latest_state_ = 255;  // 255 is used as an UNKNOWN initial state

        // Ensure DDS discovery completes before tests start to prevent lost messages
        spin_for_milliseconds(150);
    }

    void TearDown() override {
        node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
        node_.reset();
        test_node_.reset();
    }

    /**
     * @brief Utility function to deterministically spin nodes and allow timers/callbacks to fire.
     * @param duration_ms Time to spin in milliseconds.
     */
    void spin_for_milliseconds(int duration_ms) {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_->get_node_base_interface());
        executor.add_node(test_node_->get_node_base_interface());

        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(
                duration_ms))
        {
            executor.spin_some(10ms);
            std::this_thread::sleep_for(10ms);
        }
    }

    // Node under test
    std::shared_ptr<mona_control::TwistMuxNode> node_;

    // Stimulator node and interfaces
    std::shared_ptr<rclcpp::Node> test_node_;
    rclcpp::Publisher<mona_msgs::msg::FdirState>::SharedPtr fdir_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr smooth_sub_;
    rclcpp::Subscription<mona_msgs::msg::TwistMuxState>::SharedPtr state_sub_;

    // Monitored outputs
    geometry_msgs::msg::Twist latest_cmd_;
    uint8_t latest_state_;
    bool cmd_received_;
};

/* ========================================================================== */
/*                                  TEST CASES                                */
/* ========================================================================== */

// Test 1: Verify the multiplexer prioritizes manual teleop correctly
TEST_F(TwistMuxSafetyFixture, TeleopPassesThroughWhenNominal) {
    // Arrange: System is healthy
    mona_msgs::msg::FdirState health_msg;
    health_msg.current_state = mona_msgs::msg::FdirState::STATE_SOFTWARE_OK;
    fdir_pub_->publish(health_msg);
    spin_for_milliseconds(50);

    // Act: Send a teleop command (must be within limits <= 1.0)
    geometry_msgs::msg::Twist teleop_msg;
    teleop_msg.linear.x  = 0.8;
    teleop_msg.linear.y  = 0.0;
    teleop_msg.angular.z = -0.5;
    teleop_pub_->publish(teleop_msg);
    spin_for_milliseconds(150);

    // Assert: Command should pass through and status should be MANUAL
    ASSERT_TRUE(cmd_received_) << "Did not receive any smoothed twist message!";
    EXPECT_DOUBLE_EQ(latest_cmd_.linear.x, 0.8);
    EXPECT_DOUBLE_EQ(latest_cmd_.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(latest_cmd_.angular.z, -0.5);
    EXPECT_EQ(latest_state_, mona_msgs::msg::TwistMuxState::MANUAL);
}

// Test 2: Verify the multiplexer engages strict blockade on PROTECTIVE_STOP
TEST_F(TwistMuxSafetyFixture, BlocksCommandsOnProtectiveStop) {
    // Arrange: Trigger a protective stop (e.g., LiDAR failure)
    mona_msgs::msg::FdirState health_msg;
    health_msg.current_state = mona_msgs::msg::FdirState::STATE_PROTECTIVE_STOP;
    fdir_pub_->publish(health_msg);
    spin_for_milliseconds(50);

    // Act: Attempt to move the robot via teleop
    geometry_msgs::msg::Twist teleop_msg;
    teleop_msg.linear.x = 2.0;
    teleop_pub_->publish(teleop_msg);
    spin_for_milliseconds(150);

    // Assert: Output should be zeroed out and status is BLOCKED
    ASSERT_TRUE(cmd_received_);
    EXPECT_DOUBLE_EQ(
        latest_cmd_.linear.x,
        0.0) << "Velocity was not suppressed during PROTECTIVE_STOP!";
    EXPECT_DOUBLE_EQ(latest_cmd_.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(latest_cmd_.angular.z, 0.0);
    EXPECT_EQ(latest_state_, mona_msgs::msg::TwistMuxState::BLOCKED_BY_SAFETY);
}

// Test 3: Verify the multiplexer blocks commands on RECONFIGURED (e.g., PRIMARY component Lost)
TEST_F(TwistMuxSafetyFixture, BlocksCommandsOnReconfiguredState) {
    // Arrange: Trigger a reconfiguration state
    mona_msgs::msg::FdirState health_msg;
    health_msg.current_state = mona_msgs::msg::FdirState::STATE_RECONFIGURED;
    fdir_pub_->publish(health_msg);
    spin_for_milliseconds(50);

    // Act: Attempt to move the robot via teleop
    geometry_msgs::msg::Twist teleop_msg;
    teleop_msg.linear.x = 1.0;
    teleop_pub_->publish(teleop_msg);
    spin_for_milliseconds(150);

    // Assert: Output should be zeroed out and status is BLOCKED
    ASSERT_TRUE(cmd_received_);
    EXPECT_DOUBLE_EQ(
        latest_cmd_.linear.x,
        0.0) << "Velocity was not suppressed during RECONFIGURED!";
    EXPECT_DOUBLE_EQ(latest_cmd_.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(latest_cmd_.angular.z, 0.0);
    EXPECT_EQ(latest_state_, mona_msgs::msg::TwistMuxState::BLOCKED_BY_SAFETY);
}

// Test 4: Verify recovery from EMERGENCY state back to Nominal
TEST_F(TwistMuxSafetyFixture, RecoversFromEmergencyState) {
    // Arrange: Go into EMERGENCY, then recover to SOFTWARE_OK
    mona_msgs::msg::FdirState emergency_msg;
    emergency_msg.current_state = mona_msgs::msg::FdirState::STATE_EMERGENCY;
    fdir_pub_->publish(emergency_msg);
    spin_for_milliseconds(50);

    EXPECT_EQ(latest_state_, mona_msgs::msg::TwistMuxState::BLOCKED_BY_SAFETY);

    mona_msgs::msg::FdirState ok_msg;
    ok_msg.current_state = mona_msgs::msg::FdirState::STATE_SOFTWARE_OK;
    fdir_pub_->publish(ok_msg);
    spin_for_milliseconds(50);

    // Act: Send teleop command post-recovery
    geometry_msgs::msg::Twist teleop_msg;
    teleop_msg.linear.x = 0.8;
    teleop_pub_->publish(teleop_msg);
    spin_for_milliseconds(150);

    // Assert: Robot should move again
    EXPECT_DOUBLE_EQ(latest_cmd_.linear.x, 0.8) << "Failed to recover from EMERGENCY state!";
    EXPECT_EQ(latest_state_, mona_msgs::msg::TwistMuxState::MANUAL);
}

// Test 5: Verify safety unblock cannot trip the fatal state-machine guard
TEST_F(TwistMuxSafetyFixture, SafetyUnblockFromBlockedFallsBackToIdle) {
    mona_msgs::msg::FdirState ok_msg;
    ok_msg.current_state = mona_msgs::msg::FdirState::STATE_SOFTWARE_OK;
    fdir_pub_->publish(ok_msg);
    spin_for_milliseconds(50);

    geometry_msgs::msg::Twist teleop_msg;
    teleop_msg.linear.x = 0.8;
    teleop_pub_->publish(teleop_msg);
    spin_for_milliseconds(50);
    EXPECT_EQ(latest_state_, mona_msgs::msg::TwistMuxState::MANUAL);

    mona_msgs::msg::FdirState emergency_msg;
    emergency_msg.current_state = mona_msgs::msg::FdirState::STATE_EMERGENCY;
    fdir_pub_->publish(emergency_msg);
    spin_for_milliseconds(50);
    EXPECT_EQ(latest_state_, mona_msgs::msg::TwistMuxState::BLOCKED_BY_SAFETY);

    fdir_pub_->publish(ok_msg);

    EXPECT_NO_THROW(spin_for_milliseconds(100));
    EXPECT_EQ(latest_state_, mona_msgs::msg::TwistMuxState::IDLE);
    EXPECT_DOUBLE_EQ(latest_cmd_.linear.x, 0.0);
}

// Test 6: Verify velocity is clamped to NORMAL limits
TEST_F(TwistMuxSafetyFixture, RespectsNormalVelocityLimits) {
    // Arrange: Set node parameters for max speed (simulate what YAML would load)
    node_->set_parameter(rclcpp::Parameter("max_speed_normal", 1.0));

    mona_msgs::msg::FdirState health_msg;
    health_msg.current_state = mona_msgs::msg::FdirState::STATE_SOFTWARE_OK;
    fdir_pub_->publish(health_msg);
    spin_for_milliseconds(50);

    // Act: Send a teleop command exceeding the normal limit
    geometry_msgs::msg::Twist teleop_msg;
    teleop_msg.linear.x = 5.0;
    teleop_pub_->publish(teleop_msg);

    // Spin long enough for EMA to theoretically ramp up to 5.0
    spin_for_milliseconds(200);

    // Assert: Output should be clamped to 1.0
    ASSERT_TRUE(cmd_received_);
    EXPECT_LE(latest_cmd_.linear.x, 1.001) << "Velocity exceeded NORMAL maximum limit!";
}

// Test 7: Verify velocity is strictly clamped when system enters DEGRADED state
TEST_F(TwistMuxSafetyFixture, ClampsVelocityInDegradedMode) {
    // Arrange: Set parameters
    node_->set_parameter(rclcpp::Parameter("max_speed_normal", 1.0));
    node_->set_parameter(rclcpp::Parameter("max_speed_degraded", 0.3));

    // Put system into DEGRADED state (e.g., secondary sensor failure)
    mona_msgs::msg::FdirState health_msg;
    health_msg.current_state = mona_msgs::msg::FdirState::STATE_DEGRADED;
    fdir_pub_->publish(health_msg);
    spin_for_milliseconds(50);

    // Act: Send a teleop command at full normal speed (1.0)
    geometry_msgs::msg::Twist teleop_msg;
    teleop_msg.linear.x = 1.0;
    teleop_pub_->publish(teleop_msg);
    spin_for_milliseconds(200);

    // Assert: Output should be strictly clamped to DEGRADED limit (0.3)
    ASSERT_TRUE(cmd_received_);
    EXPECT_LE(latest_cmd_.linear.x, 0.301) << "Velocity exceeded DEGRADED maximum limit!";
}

// Test 8: Verify EMA (Exponential Moving Average) ramp-up functionality
TEST_F(TwistMuxSafetyFixture, VerifiesEmaRampUp) {
    // Arrange: Recreate the node with a strong EMA smoothing filter
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);

    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
        {"ema_alpha", 0.1},  // 10% step per control loop tick
        {"max_speed_normal", 1.0}
    });
    node_ = std::make_shared<mona_control::TwistMuxNode>(options);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    spin_for_milliseconds(100);
    cmd_received_ = false;

    mona_msgs::msg::FdirState health_msg;
    health_msg.current_state = mona_msgs::msg::FdirState::STATE_SOFTWARE_OK;
    fdir_pub_->publish(health_msg);
    spin_for_milliseconds(50);

    // Act: Send a sudden high-speed command
    geometry_msgs::msg::Twist teleop_msg;
    teleop_msg.linear.x = 1.0;
    teleop_pub_->publish(teleop_msg);
    spin_for_milliseconds(30);  // Spin enough for exactly a few timer ticks (timer is 10ms)

    // Assert: Because of EMA (alpha=0.1), the first output should be ~0.1, not 1.0
    ASSERT_TRUE(cmd_received_) << "Command lost! Likely due to DDS discovery delay.";
    EXPECT_NEAR(
        latest_cmd_.linear.x, 0.25,
        0.15) << "EMA filter is not smoothing the acceleration!";
}
