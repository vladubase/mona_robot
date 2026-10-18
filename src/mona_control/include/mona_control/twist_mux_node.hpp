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


#ifndef MONA_CONTROL__TWIST_MUX_NODE_HPP_
#define MONA_CONTROL__TWIST_MUX_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mona_msgs/msg/fdir_state.hpp"
#include "mona_msgs/msg/safety_state.hpp"
#include "mona_msgs/msg/twist_mux_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace mona_control
{
// Multiplexer priority and health states
enum class TwistMuxState {
	UCFG,       // Unconfigured
	IDLE,       // No active commands received
	MANUAL,     // Human teleoperation active (High Priority)
	AUTONOMOUS, // Server/Navigation control active (Standard Priority)
	BLOCKED     // Interlocked by the safety system (E-STOP)
};

// Alias for standard lifecycle return types
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TwistMuxNode : public rclcpp_lifecycle::LifecycleNode {
public:
	explicit TwistMuxNode(const rclcpp::NodeOptions &options);
	~TwistMuxNode();

protected:
	CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
	CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
	std::string mux_state_to_string(TwistMuxState state);

	void nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void fdir_state_callback(const mona_msgs::msg::FdirState::SharedPtr msg);
	void safety_state_callback(const mona_msgs::msg::SafetyState::SharedPtr msg);
	void control_loop();
	void publish_state(TwistMuxState state);

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
	rclcpp::Subscription<mona_msgs::msg::FdirState>::SharedPtr fdir_state_sub_;
	rclcpp::Subscription<mona_msgs::msg::SafetyState>::SharedPtr safety_state_sub_;

	rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr smooth_pub_;
	rclcpp_lifecycle::LifecyclePublisher<mona_msgs::msg::TwistMuxState>::SharedPtr state_pub_;

	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
	rclcpp::TimerBase::SharedPtr watchdog_timer_;

	std::mutex mux_mutex_;
	geometry_msgs::msg::Twist target_vel_;
	geometry_msgs::msg::Twist current_vel_;
	geometry_msgs::msg::Twist last_teleop_msg_;
	geometry_msgs::msg::Twist last_nav_msg_;

	rclcpp::Time last_teleop_time_;
	rclcpp::Time last_nav_time_;

	TwistMuxState current_state_;
	std::atomic<bool> is_safety_blocked_{false};
	std::atomic<bool> is_fdir_blocked_{false};
	std::atomic<bool> is_estop_blocked_{false};

	double manual_timeout_{0.0};
	double cmd_timeout_{0.0};
	double ema_alpha_{0.0};

	double max_speed_normal_{0.0};
	double max_speed_degraded_{0.0};
	std::atomic<bool> is_degraded_{false};
	std::atomic<bool> is_shutting_down_{false};
};
}  // namespace mona_control

#endif  // MONA_CONTROL__TWIST_MUX_NODE_HPP_
