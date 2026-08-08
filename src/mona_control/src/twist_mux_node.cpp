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


#include "mona_control/twist_mux_node.hpp"

using namespace std::chrono_literals;

namespace mona_control
{
TwistMuxNode::TwistMuxNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("twist_mux_node", options), last_teleop_time_(this->now()),
      last_nav_time_(this->now()), current_state_(TwistMuxState::UCFG), manual_timeout_(2.0),
      cmd_timeout_(0.5), ema_alpha_(0.1) {
    this->declare_parameter("command_timeout", 0.5);
    this->declare_parameter("manual_takeover_time", 2.0);
    this->declare_parameter("ema_alpha", 0.1);
    this->declare_parameter("max_speed_normal", 1.0);
    this->declare_parameter("max_speed_degraded", 0.3);
}

TwistMuxNode::~TwistMuxNode() {
    // CRITICAL FAULT MITIGATION: Explicitly cancel high-frequency
    // timers prior to object destruction. Prevents the ROS 2 MultiThreadedExecutor
    // from triggering callbacks on a deallocated 'this' pointer when the component
    // is abruptly unloaded (Node Crash scenario).
    is_shutting_down_ = true;

    if (watchdog_timer_) {
        watchdog_timer_->cancel();
        watchdog_timer_.reset();
    }

    // Detach action clients and subscriptions explicitly to ensure absolute memory safety
    if (nav_client_) {
        nav_client_.reset();
    }
}

CallbackReturn TwistMuxNode::on_configure(const rclcpp_lifecycle::State &) {
    cmd_timeout_        = this->get_parameter("command_timeout").as_double();
    manual_timeout_     = this->get_parameter("manual_takeover_time").as_double();
    ema_alpha_          = this->get_parameter("ema_alpha").as_double();
    max_speed_normal_   = this->get_parameter("max_speed_normal").as_double();
    max_speed_degraded_ = this->get_parameter("max_speed_degraded").as_double();

    // Initialize Reentrant group to prevent deadlocks during blocking service calls
    auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group;

    // --- PUBLISHERS --- //
    smooth_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 10);
    state_pub_  = this->create_publisher<mona_msgs::msg::TwistMuxState>("mux_state", 10);

    // --- SUBSCRIPTIONS --- //
    // Teleoperation overrides (High priority)
    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_teleop", 10, std::bind(
            &TwistMuxNode::teleop_callback, this,
            std::placeholders::_1), sub_opts);

    // Navigation and server routing (Standard priority)
    nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_nav", 10, std::bind(
            &TwistMuxNode::nav_callback, this,
            std::placeholders::_1), sub_opts);

    // Subscribe to global FDIR status to monitor system health
    fdir_state_sub_ = this->create_subscription<mona_msgs::msg::FdirState>(
        "system/health_state", 10, std::bind(
            &TwistMuxNode::fdir_state_callback, this,
            std::placeholders::_1), sub_opts);

    // Subscribe to local safety node to monitor hardware E-STOP conditions
    safety_state_sub_ = this->create_subscription<mona_msgs::msg::SafetyState>(
        "system/safety_state", 10, std::bind(
            &TwistMuxNode::safety_state_callback, this,
            std::placeholders::_1), sub_opts);

    // --- ACTION CLIENTS --- //
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose", callback_group);

    watchdog_timer_ = this->create_wall_timer(
        10ms, std::bind(&TwistMuxNode::control_loop, this), callback_group);

    RCLCPP_INFO(get_logger(), "TwistMux CONFIGURED. EMA Alpha: %.2f", ema_alpha_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn TwistMuxNode::on_activate(const rclcpp_lifecycle::State &) {
    smooth_pub_->on_activate();
    state_pub_->on_activate();

    // Initialize timestamps in the past to prevent artificial lockouts during cold boot
    rclcpp::Time past_time = this->now() - rclcpp::Duration::from_seconds(manual_timeout_ + 1.0);
    last_teleop_time_      = past_time;
    last_nav_time_         = past_time;

    current_state_ = TwistMuxState::IDLE;

    RCLCPP_INFO(get_logger(), "TwistMux ACTIVE.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn TwistMuxNode::on_deactivate(const rclcpp_lifecycle::State &) {
    smooth_pub_->on_deactivate();
    state_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn TwistMuxNode::on_cleanup(const rclcpp_lifecycle::State &) {
    // Cancel timer first to avoid Race Conditions and Segfaults
    if (watchdog_timer_ != nullptr) {
        watchdog_timer_->cancel();
        watchdog_timer_.reset();
    }

    // Safely reset all other pointers
    if (nav_sub_ != nullptr) {nav_sub_.reset();}
    if (teleop_sub_ != nullptr) {teleop_sub_.reset();}
    if (fdir_state_sub_ != nullptr) {fdir_state_sub_.reset();}
    if (safety_state_sub_ != nullptr) {safety_state_sub_.reset();}
    if (state_pub_ != nullptr) {state_pub_.reset();}
    if (smooth_pub_ != nullptr) {smooth_pub_.reset();}
    if (nav_client_ != nullptr) {nav_client_.reset();}

    return CallbackReturn::SUCCESS;
}

CallbackReturn TwistMuxNode::on_shutdown(const rclcpp_lifecycle::State &) {
    return CallbackReturn::SUCCESS;
}

void TwistMuxNode::publish_state(TwistMuxState state) {
    mona_msgs::msg::TwistMuxState msg;

    switch (state) {
        case TwistMuxState::IDLE:
            msg.current_state = mona_msgs::msg::TwistMuxState::IDLE;
            break;
        case TwistMuxState::MANUAL:
            msg.current_state = mona_msgs::msg::TwistMuxState::MANUAL;
            break;
        case TwistMuxState::AUTONOMOUS:
            msg.current_state = mona_msgs::msg::TwistMuxState::AUTONOMOUS;
            break;
        case TwistMuxState::BLOCKED:
            msg.current_state = mona_msgs::msg::TwistMuxState::BLOCKED_BY_SAFETY;
            break;
        default:
            msg.current_state = mona_msgs::msg::TwistMuxState::BLOCKED_BY_SAFETY;
            break;
    }

    if (state_pub_->is_activated()) {
        state_pub_->publish(msg);
    }
}

// Callback to monitor active FDIR health states
void TwistMuxNode::fdir_state_callback(const mona_msgs::msg::FdirState::SharedPtr msg) {
    // Track DEGRADED state to trigger velocity clamping
    is_degraded_ = (msg->current_state == mona_msgs::msg::FdirState::STATE_DEGRADED);

    is_fdir_blocked_ = (
        msg->current_state == mona_msgs::msg::FdirState::STATE_EMERGENCY ||
        msg->current_state == mona_msgs::msg::FdirState::STATE_PROTECTIVE_STOP ||
        msg->current_state == mona_msgs::msg::FdirState::STATE_RECONFIGURED);

    is_safety_blocked_ = (is_fdir_blocked_ || is_estop_blocked_);
}

// Callback to monitor explicit hardware E-STOP triggers from the safety node
void TwistMuxNode::safety_state_callback(const mona_msgs::msg::SafetyState::SharedPtr msg) {
    is_estop_blocked_  = (msg->current_state == mona_msgs::msg::SafetyState::EMERGENCY);
    is_safety_blocked_ = (is_fdir_blocked_ || is_estop_blocked_);
}

void TwistMuxNode::teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (is_shutting_down_.load()) {return;}
    std::lock_guard<std::mutex> lock(mux_mutex_);
    if (is_safety_blocked_) {return;}

    // Filter zero-velocity spam emitted by idle gamepads
    bool is_zero_cmd =
        (std::hypot(msg->linear.x, msg->linear.y) < 0.001 && std::abs(msg->angular.z) < 0.001);

    if (is_zero_cmd) {
        // Store the zero command for smooth deceleration, BUT do not extend the Nav2 lockout timer
        last_teleop_msg_ = *msg;
    } else {
        last_teleop_time_ = this->now();

        // Manual takeover: Cancel the active Nav2 goal to prevent immediate resumption
        if (current_state_ == TwistMuxState::AUTONOMOUS) {
            if (nav_client_ && nav_client_->action_server_is_ready()) {
                nav_client_->async_cancel_all_goals();
                RCLCPP_WARN(get_logger(), "MANUAL TAKEOVER: Canceling active Nav2 goal!");
            }
        }
        current_state_   = TwistMuxState::MANUAL;
        last_teleop_msg_ = *msg;
    }
}

void TwistMuxNode::nav_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (is_shutting_down_.load()) {return;}
    std::lock_guard<std::mutex> lock(mux_mutex_);
    if (is_safety_blocked_) {return;}

    last_nav_time_ = this->now();

    // Block navigation commands if the teleop interface was recently active
    if (((this->now() - last_teleop_time_).seconds()) < manual_timeout_) {return;}

    current_state_ = TwistMuxState::AUTONOMOUS;
    last_nav_msg_  = *msg;
}

void TwistMuxNode::control_loop() {
    if (!smooth_pub_->is_activated()) {return;}

    std::lock_guard<std::mutex> lock(mux_mutex_);
    auto   now                = this->now();
    double time_since_teleop  = (now - last_teleop_time_).seconds();
    double time_since_nav     = (now - last_nav_time_).seconds();
    bool   is_safe_idle_state =
        (current_state_ == TwistMuxState::BLOCKED ||
        current_state_ == TwistMuxState::IDLE ||
        current_state_ == TwistMuxState::UCFG);

    // Enforce the BLOCKED safety state regardless of incoming inputs
    if (is_safety_blocked_) {
        current_state_ = TwistMuxState::BLOCKED;
        target_vel_    = geometry_msgs::msg::Twist();   // Force zero velocity
    } else if (is_safe_idle_state) {
        current_state_ = TwistMuxState::IDLE;
        target_vel_    = geometry_msgs::msg::Twist();
    } else if ((time_since_teleop > cmd_timeout_) && (time_since_nav > cmd_timeout_)) {
        current_state_ = TwistMuxState::IDLE;
        target_vel_    = geometry_msgs::msg::Twist();
    } else {
        // Active control state routing
        if (current_state_ == TwistMuxState::MANUAL) {
            target_vel_ = last_teleop_msg_;
        } else if (current_state_ == TwistMuxState::AUTONOMOUS) {
            target_vel_ = last_nav_msg_;
        } else {
            // FATAL FAULT: State machine corrupted.
            // Throwing an exception to trigger FDIR Hardware Recovery and Zero Velocity Override.
            RCLCPP_FATAL(
                get_logger(),
                "CRITICAL: Undefined TwistMux state! Committing suicide to trigger FDIR.");
            throw std::runtime_error("TwistMuxNode: Undefined state machine state detected.");
        }
    }

    publish_state(current_state_);

    // Exponential Moving Average (EMA) Filter for smooth acceleration/deceleration
    current_vel_.linear.x = ema_alpha_ * target_vel_.linear.x + (1.0 - ema_alpha_) *
        current_vel_.linear.x;
    current_vel_.linear.y = ema_alpha_ * target_vel_.linear.y + (1.0 - ema_alpha_) *
        current_vel_.linear.y;
    current_vel_.angular.z = ema_alpha_ * target_vel_.angular.z + (1.0 - ema_alpha_) *
        current_vel_.angular.z;

    // Dynamic clamping based on FDIR state (Hard Real-Time limits enforcement)
    double current_max_speed = is_degraded_ ? max_speed_degraded_ : max_speed_normal_;

    current_vel_.linear.x =
        std::clamp(current_vel_.linear.x, -current_max_speed, current_max_speed);
    current_vel_.linear.y =
        std::clamp(current_vel_.linear.y, -current_max_speed, current_max_speed);
    current_vel_.angular.z = std::clamp(
        current_vel_.angular.z, -current_max_speed,
        current_max_speed);

    smooth_pub_->publish(current_vel_);
}
}  // namespace mona_control

// Register the component within the ROS 2 infrastructure
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mona_control::TwistMuxNode)
