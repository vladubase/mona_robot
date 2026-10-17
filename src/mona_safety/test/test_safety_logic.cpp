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
#include <cmath>
#include <cstdlib>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "mona_safety/safety_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

/**
 * @class   SafetyLogicFixture
 * @brief   Test fixture for verifying hardware interlocks,
 *          emergency stops, and hard real-time velocity limits in the SafetyNode.
 */
class SafetyLogicFixture : public ::testing::Test {
protected:
    // --- Global Setup/Teardown for the Test Suite ---
    static void SetUpTestSuite() {
        setenv("ROS_DOMAIN_ID", "12", 1);
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    // --- Per-Test Setup/Teardown ---
    void SetUp() override {
        // 1. Initialize node with test-specific hard limits
        rclcpp::NodeOptions options;
        options.parameter_overrides(
        {
            {"max_speed_normal", 1.0},
            {"max_speed_degraded", 0.3}
        });

        node_ = std::make_shared<mona_safety::SafetyNode>(options);

        // 2. Transition node to ACTIVE state
        node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

        // 3. Create stimulator node for publishers and service clients
        test_node_ = std::make_shared<rclcpp::Node>("safety_test_stimulator");

        // Publisher to simulate commands coming from TwistMux
        cmd_pub_  = test_node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 10);
        fdir_pub_ = test_node_->create_publisher<mona_msgs::msg::FdirState>(
            "system/health_state", 10);

        // Service clients for E-STOP interactions
        estop_client_       = test_node_->create_client<std_srvs::srv::Trigger>("emergency_stop");
        estop_reset_client_ = test_node_->create_client<std_srvs::srv::Trigger>(
            "emergency_stop_reset");

        // 4. Subscribers to monitor output to physical hardware
        motor_sub_ = test_node_->create_subscription<geometry_msgs::msg::Twist>(
            "hardware/motor_cmd", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_motor_cmd_ = *msg;
                cmd_received_     = true;
            });

        contactor_sub_ = test_node_->create_subscription<std_msgs::msg::Bool>(
            "hardware/contactors", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                latest_contactor_state_ = msg->data;
            });

        // Initialize state
        cmd_received_                     = false;
        latest_contactor_state_           = false;
        current_fdir_state_.current_state = mona_msgs::msg::FdirState::STATE_SOFTWARE_OK;

        fdir_timer_ = test_node_->create_wall_timer(
            50ms,
            [this]() {
                fdir_pub_->publish(current_fdir_state_);
            }
        );

        // Ensure service servers are ready before tests begin
        estop_client_->wait_for_service(std::chrono::seconds(1));
        estop_reset_client_->wait_for_service(std::chrono::seconds(1));

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
        while (std::chrono::steady_clock::now() - start_time <
            std::chrono::milliseconds(duration_ms))
        {
            executor.spin_some(10ms);
            std::this_thread::sleep_for(10ms);
        }
    }

    std::shared_ptr<mona_safety::SafetyNode> node_;
    std::shared_ptr<rclcpp::Node> test_node_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<mona_msgs::msg::FdirState>::SharedPtr fdir_pub_;
    rclcpp::TimerBase::SharedPtr fdir_timer_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estop_reset_client_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contactor_sub_;

    mona_msgs::msg::FdirState current_fdir_state_;
    geometry_msgs::msg::Twist latest_motor_cmd_;
    bool latest_contactor_state_;
    bool cmd_received_;
};

/* ========================================================================== */
/*                                  TEST CASES                                */
/* ========================================================================== */

// Test 1: Verify contactors are closed automatically when activated
TEST_F(SafetyLogicFixture, ContactorsClosedOnActivation) {
    // Arrange: Node is already activated in SetUp(), FDIR heartbeat is running

    // Act: Spin to allow DDS discovery and initial state publishing
    spin_for_milliseconds(150);

    // Assert: Contactors must be engaged (true) for normal operation
    EXPECT_TRUE(latest_contactor_state_) << "Contactors did not close upon activation!";
}

// Test 2: Verify Hard Velocity Limits (Safety bounds enforcement)
TEST_F(SafetyLogicFixture, EnforcesHardVelocityLimits) {
    // Arrange: Wait for network discovery
    spin_for_milliseconds(150);

    // Act: Send an extremely dangerous speed command exceeding normal limits
    geometry_msgs::msg::Twist input_msg;
    input_msg.linear.x  = 10.0;
    input_msg.linear.y  = -2.0;
    input_msg.angular.z = -5.0;

    cmd_pub_->publish(input_msg);
    spin_for_milliseconds(100);

    // Assert: The planar speed (vector magnitude) is limited to max_speed_normal (1.0 m/s),
    // the commanded direction is preserved, and the yaw rate is limited to
    // max_angular_speed_normal (1.0 rad/s).
    ASSERT_TRUE(cmd_received_);
    const double planar_speed = std::hypot(latest_motor_cmd_.linear.x, latest_motor_cmd_.linear.y);
    EXPECT_NEAR(planar_speed, 1.0, 1e-9) << "Planar speed exceeded max_speed_normal!";
    EXPECT_NEAR(
        latest_motor_cmd_.linear.y / latest_motor_cmd_.linear.x, -0.2, 1e-9)
        << "Commanded direction was not preserved!";
    EXPECT_DOUBLE_EQ(latest_motor_cmd_.angular.z, -1.0) << "Angular velocity exceeded limit!";
}

// Test 2b: Verify that a diagonal command cannot exceed the planar speed limit
TEST_F(SafetyLogicFixture, LimitsDiagonalPlanarSpeed) {
    spin_for_milliseconds(150);

    // Act: Command 1.0 m/s on both axes (1.41 m/s resultant on a holonomic base)
    geometry_msgs::msg::Twist input_msg;
    input_msg.linear.x = 1.0;
    input_msg.linear.y = 1.0;

    cmd_pub_->publish(input_msg);
    spin_for_milliseconds(100);

    // Assert: The resultant speed equals the limit, not sqrt(2) times the limit
    ASSERT_TRUE(cmd_received_);
    EXPECT_NEAR(
        std::hypot(latest_motor_cmd_.linear.x, latest_motor_cmd_.linear.y), 1.0, 1e-9)
        << "Diagonal motion exceeded max_speed_normal!";
}

// Test 2c: Verify that non-finite commands are replaced by a stop
TEST_F(SafetyLogicFixture, RejectsNonFiniteCommand) {
    spin_for_milliseconds(150);

    // Act: Command NaN, which would pass every comparison-based limit
    geometry_msgs::msg::Twist input_msg;
    input_msg.linear.x = std::numeric_limits<double>::quiet_NaN();

    cmd_pub_->publish(input_msg);
    spin_for_milliseconds(100);

    // Assert: The motor command is a finite stop
    ASSERT_TRUE(cmd_received_);
    EXPECT_DOUBLE_EQ(latest_motor_cmd_.linear.x, 0.0) << "Non-finite command reached the motors!";
    EXPECT_DOUBLE_EQ(latest_motor_cmd_.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(latest_motor_cmd_.angular.z, 0.0);
}

// Test 3: Verify E-STOP Service triggers hardware disconnect
TEST_F(SafetyLogicFixture, EstopServiceOpensContactors) {
    // Arrange: Ensure system is running and contactors are closed
    spin_for_milliseconds(150);

    // Act: Trigger software E-STOP via service call
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result  = estop_client_->async_send_request(request);
    spin_for_milliseconds(200);

    // Assert: Hardware contactors must be physically disconnected (false)
    EXPECT_FALSE(latest_contactor_state_) << "Contactors remained closed during E-STOP!";
}

// Test 4: Verify E-STOP overrides and zeroes out incoming velocity commands
TEST_F(SafetyLogicFixture, EstopZeroesOutVelocity) {
    // Arrange: Ensure system is running, then trigger E-STOP
    spin_for_milliseconds(150);
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    estop_client_->async_send_request(request);
    spin_for_milliseconds(150);

    // Act: Try to move the robot while E-STOP is active
    geometry_msgs::msg::Twist input_msg;
    input_msg.linear.x = 1.0;
    cmd_pub_->publish(input_msg);
    spin_for_milliseconds(100);

    // Assert: Regardless of input, output to motors must be absolute zero
    ASSERT_TRUE(cmd_received_);
    EXPECT_DOUBLE_EQ(latest_motor_cmd_.linear.x, 0.0) << "Motion NOT allowed during E-STOP!";
}

// Test 5: Verify E-STOP Reset restores system to operational state
TEST_F(SafetyLogicFixture, EstopResetRestoresOperation) {
    // Arrange: Put system into E-STOP mode
    spin_for_milliseconds(150);
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    estop_client_->async_send_request(request);
    spin_for_milliseconds(150);
    EXPECT_FALSE(latest_contactor_state_);

    // Act: Trigger Reset via service
    auto reset_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    estop_reset_client_->async_send_request(reset_req);
    spin_for_milliseconds(150);

    // Assert: Contactors should close again, restoring power to drives
    EXPECT_TRUE(latest_contactor_state_) << "Contactors did not close after E-STOP Reset!";
}

// Test 6: Verify that a reset alone never restores power without FDIR clearance
TEST_F(SafetyLogicFixture, EstopResetRequiresFdirClearance) {
    // Arrange: Trigger the E-STOP, then stop the simulated FDIR health stream
    spin_for_milliseconds(150);
    estop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    spin_for_milliseconds(150);
    fdir_timer_->cancel();
    spin_for_milliseconds(100);

    // Act: Reset the E-STOP while no FDIR clearance arrives
    estop_reset_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    spin_for_milliseconds(200);

    // Assert: The contactors stay open and motion commands are replaced by a stop
    EXPECT_FALSE(latest_contactor_state_) << "Reset closed the contactors without FDIR clearance!";
    geometry_msgs::msg::Twist input_msg;
    input_msg.linear.x = 0.5;
    cmd_pub_->publish(input_msg);
    spin_for_milliseconds(100);
    ASSERT_TRUE(cmd_received_);
    EXPECT_DOUBLE_EQ(latest_motor_cmd_.linear.x, 0.0) << "Motion allowed without FDIR clearance!";

    // Act: FDIR clearance resumes
    fdir_timer_->reset();
    spin_for_milliseconds(150);

    // Assert: Power is restored only after FDIR clearance
    EXPECT_TRUE(latest_contactor_state_) << "Contactors did not close after FDIR clearance!";
}

// Test 7: Verify that an unknown FDIR state halts motion and opens the contactors
TEST_F(SafetyLogicFixture, UnknownFdirStateHaltsMotion) {
    spin_for_milliseconds(150);
    ASSERT_TRUE(latest_contactor_state_);

    // Act: FDIR reports an undefined state ID
    current_fdir_state_.current_state = 99;
    spin_for_milliseconds(150);

    geometry_msgs::msg::Twist input_msg;
    input_msg.linear.x = 0.5;
    cmd_pub_->publish(input_msg);
    spin_for_milliseconds(100);

    // Assert
    EXPECT_FALSE(latest_contactor_state_) << "Contactors closed in an undefined FDIR state!";
    ASSERT_TRUE(cmd_received_);
    EXPECT_DOUBLE_EQ(
        latest_motor_cmd_.linear.x,
        0.0) << "Motion allowed in an undefined FDIR state!";
}
