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
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mona_core/fdir_manager_node.hpp"
#include "mona_msgs/msg/fdir_state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

/**
 * @class   FdirManagerFixture
 * @brief   Test fixture for verifying Fault Detection, Isolation, and Recovery
 *          orchestration within the FdirManagerNode.
 */
class FdirManagerFixture : public ::testing::Test {
protected:
    // --- Global Setup/Teardown for the Test Suite ---
    static void SetUpTestSuite() {
        setenv("ROS_DOMAIN_ID", "15", 1);
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    // --- Per-Test Setup/Teardown ---
    void SetUp() override {
        // 1. Initialize node with dynamic parameter overrides (mocking fdir_policy.yaml)
        rclcpp::NodeOptions options;
        options.parameter_overrides(
        {
            rclcpp::Parameter(
                "tracked_components",
                std::vector<std::string>{"mock_safety", "mock_mux"}),

            // Mock Safety Node (FATAL)
            rclcpp::Parameter("components.mock_safety.enabled", true),
            rclcpp::Parameter("components.mock_safety.node_name", "mock_safety_node"),
            rclcpp::Parameter("components.mock_safety.tier", "FATAL"),
            rclcpp::Parameter("components.mock_safety.reset_mechanism", "lifecycle_service"),

            // Mock Twist Mux (PRIMARY)
            rclcpp::Parameter("components.mock_mux.enabled", true),
            rclcpp::Parameter("components.mock_mux.node_name", "mock_mux_node"),
            rclcpp::Parameter("components.mock_mux.tier", "PRIMARY"),
            rclcpp::Parameter("components.mock_mux.reset_mechanism", "lifecycle_service")
        });

        node_ = std::make_shared<mona_core::FdirManagerNode>(options);

        // 2. Create stimulator node for subscriptions
        test_node_ = std::make_shared<rclcpp::Node>("fdir_test_stimulator");

        // 3. Subscribers to monitor FDIR Manager output
        health_sub_ = test_node_->create_subscription<mona_msgs::msg::FdirState>(
            "system/health_state", 10,
            [this](const mona_msgs::msg::FdirState::SharedPtr msg) {
                latest_health_state_ = msg->current_state;
                health_msg_received_ = true;
            });

        contactor_sub_ = test_node_->create_subscription<std_msgs::msg::Bool>(
            "hardware/contactors", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                latest_contactor_state_ = msg->data;
                contactor_msg_received_ = true;
            });

        // Initialize state
        health_msg_received_    = false;
        contactor_msg_received_ = false;
        latest_health_state_    = mona_msgs::msg::FdirState::STATE_SYSTEM_STARTUP;

        // Ensure DDS discovery completes before tests start
        spin_for_milliseconds(150);
    }

    void TearDown() override {
        node_.reset();
        test_node_.reset();
    }

    /**
     * @brief   Utility function to deterministically spin nodes and allow timers/callbacks to fire.
     * @param   duration_ms Time to spin in milliseconds.
     */
    void spin_for_milliseconds(int duration_ms) {
        rclcpp::executors::MultiThreadedExecutor executor;
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

    // --- Member Variables ---
    std::shared_ptr<mona_core::FdirManagerNode> node_;
    std::shared_ptr<rclcpp::Node> test_node_;

    rclcpp::Subscription<mona_msgs::msg::FdirState>::SharedPtr health_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contactor_sub_;

    uint8_t latest_health_state_;
    bool latest_contactor_state_;
    bool health_msg_received_;
    bool contactor_msg_received_;
};

/* ========================================================================== */
/*                                  TEST CASES                                */
/* ========================================================================== */

// Test 1: Verify successful node instantiation and dynamic parameter parsing
TEST_F(FdirManagerFixture, NodeInstantiation) {
    EXPECT_NE(node_, nullptr);
    EXPECT_STREQ(node_->get_name(), "mona_fdir_manager");
}

// Test 2: Verify pessimistic startup phase
// Upon boot, the manager should default to STATE_SYSTEM_STARTUP until it successfully
// pings and verifies all managed lifecycle nodes.
TEST_F(FdirManagerFixture, PessimisticStartupState) {
    // Arrange: Node is running, timers are active

    // Act: Spin executor to allow FDIR state machine to process timeouts.
    // 2 mock nodes * (50ms get + 50ms change) = 200ms per loop worst-case.
    // Spinning for 1000ms guarantees the loop completes and publishes health state.
    spin_for_milliseconds(1000);

    // Assert: We should have received a health broadcast stating the system is in STARTUP
    ASSERT_TRUE(health_msg_received_) << "No health state was published!";

    // Since our mock lifecycle nodes (mock_safety_node, mock_mux_node) are not actually
    // running in this test, the FDIR manager will ping them, timeout, and stay in STARTUP.
    EXPECT_EQ(latest_health_state_, mona_msgs::msg::FdirState::STATE_SYSTEM_STARTUP)
        << "System did not default to pessimistic startup state!";
}

/**
 * @class   FdirManagerConfigValidationFixture
 * @brief   Isolated fixture for fail-fast FDIR configuration validation.
 */
class FdirManagerConfigValidationFixture : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        setenv("ROS_DOMAIN_ID", "16", 1);
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

TEST_F(FdirManagerConfigValidationFixture, RejectsMissingTrackedComponents) {
    rclcpp::NodeOptions options;

    EXPECT_THROW(
    {
        auto node = std::make_shared<mona_core::FdirManagerNode>(options);
    },
        std::runtime_error);
}

TEST_F(FdirManagerConfigValidationFixture, RejectsEmptyNodeName) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
        rclcpp::Parameter("tracked_components", std::vector<std::string>{"broken"}),
        rclcpp::Parameter("components.broken.enabled", true),
        rclcpp::Parameter("components.broken.node_name", ""),
        rclcpp::Parameter("components.broken.tier", "FATAL"),
        rclcpp::Parameter("components.broken.reset_mechanism", "lifecycle_service")
    });

    EXPECT_THROW(
    {
        auto node = std::make_shared<mona_core::FdirManagerNode>(options);
    },
        std::runtime_error);
}

TEST_F(FdirManagerConfigValidationFixture, RejectsHardwareRelayWithoutResetTopic) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
        rclcpp::Parameter("tracked_components", std::vector<std::string>{"relay"}),
        rclcpp::Parameter("components.relay.enabled", true),
        rclcpp::Parameter("components.relay.node_name", "relay_node"),
        rclcpp::Parameter("components.relay.tier", "PRIMARY"),
        rclcpp::Parameter("components.relay.reset_mechanism", "hardware_relay"),
        rclcpp::Parameter("components.relay.reset_topic", "")
    });

    EXPECT_THROW(
    {
        auto node = std::make_shared<mona_core::FdirManagerNode>(options);
    },
        std::runtime_error);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
