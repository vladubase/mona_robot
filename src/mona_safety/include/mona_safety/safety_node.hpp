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


#ifndef MONA_SAFETY__SAFETY_NODE_HPP_
#define MONA_SAFETY__SAFETY_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "mona_msgs/msg/fdir_state.hpp"
#include "mona_msgs/msg/safety_state.hpp"
#include "mona_msgs/msg/twist_mux_state.hpp"
#include "mona_msgs/msg/safety_diagnostics.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

namespace mona_safety
{
// Alias for standard lifecycle return types
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum class SafetyState {
    NORMAL,
    DEGRADED,  // Performance issues detected (slow mode enabled)
    PROTECTIVE_STOP,  // Vehicle stopped, waiting for a PRIMARY node to reboot
    EMERGENCY  // E-STOP active (contactors physically opened)
};

enum class FdirCommand {
    SYSTEM_STARTUP,
    EMERGENCY,
    PROTECTIVE_STOP,
    DEGRADED,
    SOFTWARE_OK,
    UNKNOWN
};

class SafetyNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit SafetyNode(const rclcpp::NodeOptions &options);
    ~SafetyNode();

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

private:
    std::string safety_state_to_string(SafetyState state);

    void smoothed_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void health_state_callback(const mona_msgs::msg::FdirState::SharedPtr msg);
    void mux_state_callback(const mona_msgs::msg::TwistMuxState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void estop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void estop_reset_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void set_hardware_contactors(bool enable);
    void publish_stop();
    void publish_global_state();
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr smooth_sub_;
    rclcpp::Subscription<mona_msgs::msg::FdirState>::SharedPtr health_sub_;
    rclcpp::Subscription<mona_msgs::msg::TwistMuxState>::SharedPtr mux_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_estop_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_estop_reset_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp_lifecycle::LifecyclePublisher<mona_msgs::msg::SafetyState>::SharedPtr safety_state_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr contactor_pub_;
    diagnostic_updater::Updater diagnostic_updater_;

    // Default safe state on initialization
    SafetyState current_state_{SafetyState::PROTECTIVE_STOP};
    uint8_t last_fdir_state_{255};  // 255 as an invalid initial state
    uint8_t last_mux_state_{mona_msgs::msg::TwistMuxState::IDLE};

    std::atomic<bool> e_stop_active_{false};
    std::atomic<bool> is_processing_allowed_{false};
    bool hardware_contactors_closed_{false};

    double max_speed_normal_{0.0};  // [m/s]   planar linear speed limit
    double max_speed_degraded_{0.0};  // [m/s]   planar linear speed limit, DEGRADED
    double max_angular_speed_normal_{0.0};  // [rad/s] yaw rate limit
    double max_angular_speed_degraded_{0.0};  // [rad/s] yaw rate limit, DEGRADED
};
}  // namespace mona_safety

#endif  // MONA_SAFETY__SAFETY_NODE_HPP_
