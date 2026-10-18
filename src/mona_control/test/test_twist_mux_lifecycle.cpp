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

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mona_control/twist_mux_node.hpp"

// Test fixture to initialize the standard ROS 2 environment
class TestTwistMux : public ::testing::Test {
protected:
	static void SetUpTestSuite() {
		rclcpp::init(0, nullptr);
	}

	static void TearDownTestSuite() {
		rclcpp::shutdown();
	}
};

// Test 1: Verify initialization, name registration, and lifecycle state transitions
TEST_F(TestTwistMux, LifecycleTransition) {
	rclcpp::NodeOptions options;
	auto node = std::make_shared<mona_control::TwistMuxNode>(options);

	// Smoke Test: Verify Node Instantiation and Name
	EXPECT_NE(node, nullptr);
	EXPECT_STREQ(node->get_name(), "twist_mux_node");

	// Verify the initial state is strictly UNCONFIGURED
	EXPECT_EQ(
		node->get_current_state().id(),
		lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED
	);

	// Trigger on_configure()
	// This verifies memory allocation, parameter retrieval, and publisher/subscription setup
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
