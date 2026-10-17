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


#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include "mona_safety/safety_node.hpp"

#define COLOR_BLUE  "\033[34m"
#define COLOR_RESET "\033[0m"

namespace mona_safety
{
SafetyNode::SafetyNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("safety_node", options), diagnostic_updater_(this) {
    // The node starts in PROTECTIVE_STOP (header default); FDIR clearance is required for motion.
    this->declare_parameter("max_speed_normal", 1.0);  // [m/s]
    this->declare_parameter("max_speed_degraded", 0.3);  // [m/s]
    this->declare_parameter("max_angular_speed_normal", 1.0);  // [rad/s]
    this->declare_parameter("max_angular_speed_degraded", 0.3);  // [rad/s]

    diagnostic_updater_.setHardwareID("mona_base_controller");
    diagnostic_updater_.add("Safety Node", this, &SafetyNode::produce_diagnostics);
}

SafetyNode::~SafetyNode() {
    // A destructor must not let an exception escape: that would call std::terminate
    // without a controlled shutdown. Failures are reported on stderr, because the
    // ROS logging infrastructure may itself be the source of the exception.
    try {
        // 1. Actuate hardware safety fallback (Fail-Safe) prior to memory teardown
        set_hardware_contactors(false);

        // 2. CRITICAL FAULT MITIGATION: Terminate active service servers to prevent
        // incoming E-STOP or Reset network requests from accessing a dying component instance.
        if (srv_estop_) {
            srv_estop_.reset();
        }
        if (srv_estop_reset_) {
            srv_estop_reset_.reset();
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "[safety_node] Exception during destruction: %s\n", e.what());
    } catch (...) {
        std::fprintf(stderr, "[safety_node] Unknown exception during destruction.\n");
    }
}

std::string SafetyNode::safety_state_to_string(SafetyState state) {
    std::string mux_str = "UNKNOWN";
    switch (last_mux_state_) {
        case mona_msgs::msg::TwistMuxState::IDLE:              mux_str = "IDLE"; break;
        case mona_msgs::msg::TwistMuxState::MANUAL:            mux_str = "MANUAL"; break;
        case mona_msgs::msg::TwistMuxState::AUTONOMOUS:        mux_str = "AUTONOMOUS"; break;
        case mona_msgs::msg::TwistMuxState::BLOCKED_BY_SAFETY: mux_str = "BLOCKED"; break;
        default:                                               mux_str = "UNKNOWN"; break;
    }

    switch (state) {
        case SafetyState::NORMAL:          return "NORMAL: " + mux_str;
        case SafetyState::DEGRADED:        return "DEGRADED: " + mux_str;
        case SafetyState::PROTECTIVE_STOP: return "PROTECTIVE_STOP";
        case SafetyState::EMERGENCY:       return "EMERGENCY";
        default:                           return "UNKNOWN";
    }
}

CallbackReturn SafetyNode::on_configure(const rclcpp_lifecycle::State &) {
    max_speed_normal_           = this->get_parameter("max_speed_normal").as_double();
    max_speed_degraded_         = this->get_parameter("max_speed_degraded").as_double();
    max_angular_speed_normal_   = this->get_parameter("max_angular_speed_normal").as_double();
    max_angular_speed_degraded_ = this->get_parameter("max_angular_speed_degraded").as_double();

    // Reject an invalid limit set: each limit must be finite and positive, and the
    // DEGRADED limit must not exceed the NORMAL limit. A failed configuration keeps
    // the node unconfigured, which FDIR detects and reports.
    const auto is_valid_pair = [](double normal, double degraded) {
            return std::isfinite(normal) && std::isfinite(degraded) &&
                   normal > 0.0 && degraded > 0.0 && degraded <= normal;
        };
    if (!is_valid_pair(max_speed_normal_, max_speed_degraded_) ||
        !is_valid_pair(max_angular_speed_normal_, max_angular_speed_degraded_))
    {
        RCLCPP_FATAL(
            get_logger(),
            "Invalid velocity limits (linear %.3f/%.3f m/s, angular %.3f/%.3f rad/s). "
            "Configuration rejected.",
            max_speed_normal_, max_speed_degraded_,
            max_angular_speed_normal_, max_angular_speed_degraded_);
        return CallbackReturn::FAILURE;
    }

    // Initialize Reentrant group to prevent deadlocks during blocking service calls
    auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group;

    smooth_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_smoothed", 10,
        std::bind(&SafetyNode::smoothed_vel_callback, this, std::placeholders::_1), sub_opts);

    health_sub_ = create_subscription<mona_msgs::msg::FdirState>(
        "system/health_state", 10,
        std::bind(&SafetyNode::health_state_callback, this, std::placeholders::_1), sub_opts);

    mux_state_sub_ = create_subscription<mona_msgs::msg::TwistMuxState>(
        "mux_state", 10,
        std::bind(&SafetyNode::mux_state_callback, this, std::placeholders::_1), sub_opts);

    // Subscribe to odometry to monitor actual physical stops
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SystemDefaultsQoS(),
        std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1));

    srv_estop_ = create_service<std_srvs::srv::Trigger>(
        "emergency_stop",
        std::bind(
            &SafetyNode::estop_callback, this, std::placeholders::_1,
            std::placeholders::_2));
    srv_estop_reset_ = create_service<std_srvs::srv::Trigger>(
        "emergency_stop_reset",
        std::bind(
            &SafetyNode::estop_reset_callback, this, std::placeholders::_1,
            std::placeholders::_2)
    );

    motor_pub_        = create_publisher<geometry_msgs::msg::Twist>("hardware/motor_cmd", 10);
    safety_state_pub_ = create_publisher<mona_msgs::msg::SafetyState>("system/safety_state", 10);
    contactor_pub_    = create_publisher<std_msgs::msg::Bool>("hardware/contactors", 10);

    RCLCPP_INFO(get_logger(), "Safety Node CONFIGURED.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SafetyNode::on_activate(const rclcpp_lifecycle::State &) {
    is_processing_allowed_ = true;
    motor_pub_->on_activate();
    safety_state_pub_->on_activate();
    contactor_pub_->on_activate();
    set_hardware_contactors(false);

    RCLCPP_INFO(get_logger(), "Safety Core ACTIVE. Contactors OFF. Waiting for FDIR clearance...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SafetyNode::on_deactivate(const rclcpp_lifecycle::State &) {
    is_processing_allowed_ = false;
    set_hardware_contactors(false);

    if (motor_pub_) {motor_pub_->on_deactivate();}
    if (safety_state_pub_) {safety_state_pub_->on_deactivate();}
    if (contactor_pub_) {contactor_pub_->on_deactivate();}

    RCLCPP_WARN(get_logger(), "Safety Node DEACTIVATED. System may be unstable!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SafetyNode::on_cleanup(const rclcpp_lifecycle::State &) {
    is_processing_allowed_ = false;
    set_hardware_contactors(false);

    if (smooth_sub_) {smooth_sub_.reset();}
    if (health_sub_) {health_sub_.reset();}
    if (mux_state_sub_) {mux_state_sub_.reset();}
    if (odom_sub_) {odom_sub_.reset();}
    if (srv_estop_) {srv_estop_.reset();}
    if (srv_estop_reset_) {srv_estop_reset_.reset();}
    if (motor_pub_) {motor_pub_.reset();}
    if (safety_state_pub_) {safety_state_pub_.reset();}
    if (contactor_pub_) {contactor_pub_.reset();}

    return CallbackReturn::SUCCESS;
}

CallbackReturn SafetyNode::on_shutdown(const rclcpp_lifecycle::State &) {
    is_processing_allowed_ = false;
    set_hardware_contactors(false);

    return CallbackReturn::SUCCESS;
}

CallbackReturn SafetyNode::on_error(const rclcpp_lifecycle::State &) {
    is_processing_allowed_ = false;
    set_hardware_contactors(false);

    return CallbackReturn::SUCCESS;
}

void SafetyNode::set_hardware_contactors(bool enable) {
    if (hardware_contactors_closed_ == enable) {return;}
    hardware_contactors_closed_ = enable;

    std_msgs::msg::Bool contactor_msg;
    contactor_msg.data = enable;
    if (contactor_pub_ && contactor_pub_->is_activated()) {
        contactor_pub_->publish(contactor_msg);
    }

    if (enable) {
        RCLCPP_INFO(
            get_logger(),
            COLOR_BLUE "HARDWARE: Contactors CLOSED. Motors POWERED." COLOR_RESET);
    } else {
        RCLCPP_WARN(get_logger(), "HARDWARE: Contactors OPENED. Motors DEAD.");
    }
}

void SafetyNode::publish_stop() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x  = 0.0;
    stop_msg.linear.y  = 0.0;
    stop_msg.angular.z = 0.0;

    if (motor_pub_->is_activated()) {motor_pub_->publish(stop_msg);}
}

void SafetyNode::publish_global_state() {
    // Check if the lifecycle publisher is activated
    if (!safety_state_pub_->is_activated()) {return;}

    mona_msgs::msg::SafetyState msg;

    // Strict status priority hierarchy utilizing the state converter
    if (e_stop_active_ || current_state_ == SafetyState::EMERGENCY) {
        msg.current_state = mona_msgs::msg::SafetyState::EMERGENCY;
        msg.message       = safety_state_to_string(SafetyState::EMERGENCY);
    } else if (current_state_ == SafetyState::PROTECTIVE_STOP) {
        msg.current_state = mona_msgs::msg::SafetyState::PROTECTIVE_STOP;
        msg.message       = safety_state_to_string(SafetyState::PROTECTIVE_STOP);
    } else if (current_state_ == SafetyState::DEGRADED) {
        msg.current_state = mona_msgs::msg::SafetyState::DEGRADED;
        msg.message       = safety_state_to_string(SafetyState::DEGRADED);
    } else {
        // If hardware is operational, broadcast the NORMAL state with active control info
        msg.current_state = mona_msgs::msg::SafetyState::NORMAL;
        msg.message       = safety_state_to_string(SafetyState::NORMAL);
    }

    safety_state_pub_->publish(msg);
}

void SafetyNode::smoothed_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!is_processing_allowed_ || !motor_pub_->is_activated()) {return;}

    if (e_stop_active_ || current_state_ == SafetyState::PROTECTIVE_STOP ||
        current_state_ == SafetyState::EMERGENCY)
    {
        publish_stop();
        return;
    }

    // A non-finite component (NaN, Inf) cannot be limited meaningfully: comparisons
    // with NaN are always false, so it would pass every clamp. Such a command is
    // replaced by a stop.
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
        !std::isfinite(msg->angular.z))
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Non-finite velocity command rejected. Publishing stop.");
        publish_stop();
        return;
    }

    const bool   is_degraded   = (current_state_ == SafetyState::DEGRADED);
    const double linear_limit  = is_degraded ? max_speed_degraded_ : max_speed_normal_;
    const double angular_limit =
        is_degraded ? max_angular_speed_degraded_ : max_angular_speed_normal_;

    // Planar base: only linear.x, linear.y and angular.z are forwarded.
    geometry_msgs::msg::Twist clipped_msg;
    clipped_msg.linear.x  = msg->linear.x;
    clipped_msg.linear.y  = msg->linear.y;
    clipped_msg.angular.z = std::clamp(msg->angular.z, -angular_limit, angular_limit);

    // Limit the magnitude of the planar velocity vector. A holonomic (mecanum) base
    // can move diagonally, so limiting x and y separately would allow sqrt(2) times
    // the limit. Uniform scaling preserves the commanded direction.
    const double linear_speed = std::hypot(clipped_msg.linear.x, clipped_msg.linear.y);
    if (linear_speed > linear_limit) {
        const double scale    = linear_limit / linear_speed;
        clipped_msg.linear.x *= scale;
        clipped_msg.linear.y *= scale;
    }

    motor_pub_->publish(clipped_msg);
}

void SafetyNode::health_state_callback(const mona_msgs::msg::FdirState::SharedPtr msg) {
    if (!is_processing_allowed_) {return;}  // ZOMBIE NODE PROTECTION

    uint8_t fdir_state    = msg->current_state;
    bool    state_changed = (fdir_state != last_fdir_state_);
    last_fdir_state_      = fdir_state;

    // Execute state transition logic based on the explicit byte command
    switch (fdir_state) {
        case mona_msgs::msg::FdirState::STATE_SYSTEM_STARTUP:
            // 1. Initialization (Contactors OFF)
            current_state_ = SafetyState::PROTECTIVE_STOP;
            publish_stop();
            set_hardware_contactors(false);
            if (state_changed) {
                RCLCPP_INFO(
                    this->get_logger(),
                    COLOR_BLUE "System is initializing... Contactors OFF." COLOR_RESET);
            }
            break;

        case mona_msgs::msg::FdirState::STATE_EMERGENCY:
            // 2. Emergency (Latch E-Stop, contactors OFF)
            e_stop_active_ = true;
            current_state_ = SafetyState::EMERGENCY;
            publish_stop();
            set_hardware_contactors(false);
            if (state_changed) {
                RCLCPP_FATAL(this->get_logger(), "FDIR Commanded EMERGENCY! Contactors OFF.");
            }
            break;

        case mona_msgs::msg::FdirState::STATE_PROTECTIVE_STOP:
            // 3. Protective Stop (Power is ON, but motion is halted)
            current_state_ = SafetyState::PROTECTIVE_STOP;
            publish_stop();
            if (!e_stop_active_) {set_hardware_contactors(true);}
            if (state_changed) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "FDIR Commanded PROTECTIVE STOP. Waiting for primary sensors...");
            }
            break;

        case mona_msgs::msg::FdirState::STATE_DEGRADED:
            // 4. Degraded Mode (Operational, but with velocity restrictions)
            current_state_ = SafetyState::DEGRADED;
            if (!e_stop_active_) {
                set_hardware_contactors(true);
            }
            break;

        case mona_msgs::msg::FdirState::STATE_SOFTWARE_OK:
            // 5. Normal Operation
            if (!e_stop_active_) {
                current_state_ = SafetyState::NORMAL;
                set_hardware_contactors(true);
            } else {
                if (state_changed) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "FDIR Commanded SOFTWARE_OK, but E-STOP is latched! Ignoring.");
                }
            }
            break;

        case mona_msgs::msg::FdirState::STATE_RECONFIGURED:
            // 6. Reconfigured Mode
            // Primary component failed. Manual control allowed, autonomous blocked
            current_state_ = SafetyState::DEGRADED;
            publish_stop();  // Halt any current motion immediately on transition
            if (!e_stop_active_) {
                set_hardware_contactors(true);
            }
            if (state_changed) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "FDIR Commanded RECONFIGURED. PRIMARY COMPONENT LOST. Manual override only!");
            }
            break;

        default:
            // 7. UNKNOWN STATE (Global fallback - power off everything)
            // Motion is halted as well, so that no velocity command passes while the
            // health state is undefined (e.g. interface version mismatch).
            current_state_ = SafetyState::PROTECTIVE_STOP;
            publish_stop();
            set_hardware_contactors(false);
            if (state_changed) {
                RCLCPP_ERROR(
                    this->get_logger(), "UNKNOWN FDIR STATE ID: '%d'. Safety shutdown triggered!",
                    fdir_state);
            }
            break;
    }
}

void SafetyNode::mux_state_callback(const mona_msgs::msg::TwistMuxState::SharedPtr msg) {
    if (!is_processing_allowed_) {return;}
    last_mux_state_ = msg->current_state;
    publish_global_state();
}

void SafetyNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!is_processing_allowed_) {return;}

    double current_speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

    if (current_state_ == SafetyState::PROTECTIVE_STOP && current_speed > 0.05) {
        e_stop_active_ = true;
        current_state_ = SafetyState::EMERGENCY;
        set_hardware_contactors(false);
        publish_global_state();

        RCLCPP_FATAL(
            get_logger(), "ESCALATION: Robot moving during Soft Stop! Hard E-STOP triggered!");
    }
}

void SafetyNode::estop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!is_processing_allowed_) {return;}

    e_stop_active_ = true;
    current_state_ = SafetyState::EMERGENCY;
    publish_stop();
    set_hardware_contactors(false);
    publish_global_state();

    response->success = true;
    response->message = "EMERGENCY STOP (LEVEL 2) ACTIVATED!";
    RCLCPP_FATAL(get_logger(), "!!! LEVEL 2 E-STOP: MOTORS DE-ENERGIZED !!!");
}

void SafetyNode::estop_reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!is_processing_allowed_) {
        response->success = false;
        response->message = "Reset rejected: Safety Node is not active.";
        return;
    }

    if (current_state_ == SafetyState::EMERGENCY || e_stop_active_) {
        // A reset releases the E-STOP latch only and never enables motion by itself
        // (ISO 13850, ISO 14118). The node enters PROTECTIVE_STOP with the contactors
        // open; the next FDIR health state decides whether operation may resume.
        e_stop_active_ = false;
        current_state_ = SafetyState::PROTECTIVE_STOP;
        publish_stop();
        set_hardware_contactors(false);

        publish_global_state();

        RCLCPP_INFO(
            this->get_logger(), "E-STOP latch released. Awaiting FDIR clearance.");
        response->success = true;
        response->message = "E-STOP latch released. PROTECTIVE_STOP until FDIR clearance.";
    } else {
        response->success = false;
        response->message = "Reset ignored: System is not in EMERGENCY state.";
    }
}

void SafetyNode::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    if (hardware_contactors_closed_) {
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::OK,
            "System Normal. Contactors Closed.");
    } else {
        stat.summary(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            "System Offline. Contactors Opened.");
    }
}
}  // namespace mona_safety

// Register the component within the ROS 2 infrastructure
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mona_safety::SafetyNode)
