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


#ifndef MONA_CORE__FDIR_MANAGER_NODE_HPP_
#define MONA_CORE__FDIR_MANAGER_NODE_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "mona_msgs/msg/fdir_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace mona_core
{
/**
 * @brief   FDIR Manager class for autonomous fault detection and recovery.
 *          Orchestrates Lifecycle transitions for critical robot components.
 */
class FdirManagerNode : public rclcpp::Node {
public:
    explicit FdirManagerNode(const rclcpp::NodeOptions &options);
    ~FdirManagerNode();

    // Prevent accidental copying or moving of the safety manager (Rule of Five)
    FdirManagerNode(const FdirManagerNode &)             = delete;
    FdirManagerNode & operator=(const FdirManagerNode &) = delete;
    FdirManagerNode(FdirManagerNode &&)             = delete;
    FdirManagerNode & operator=(FdirManagerNode &&) = delete;

private:
    static constexpr int MAX_RECOVERY_RETRIES = 3;
    static constexpr int MONITOR_PERIOD_MS    = 100;

    /** @brief   Node criticality tiers according to fdir_policy.yaml */
    enum class Tier {
        FATAL,  // Triggers hardware EMERGENCY E-Stop
        PRIMARY,  // Triggers software PROTECTIVE_STOP
        AUXILIARY  // Triggers DEGRADED mode
    };

    /** @brief   Defines the deterministic mechanism utilized to reset a failed component.
     */
    enum class ResetMechanism {
        LIFECYCLE_SERVICE,  // Software-only recovery via ROS 2 lifecycle transitions
        HARDWARE_RELAY  // Physical power interruption (12V/24V) for sensors
    };

    /** @brief   Finite State Machine (FSM) phases for deterministic component recovery. */
    enum class RecoveryPhase {
        NOMINAL,  // Component operating normally
        POWER_OFF,  // Initiating power cutoff or software cleanup
        WAIT_DISCHARGE,  // Waiting for capacitors to discharge or OS to halt
        POWER_ON,  // Restoring power
        WAIT_BOOT,  // Waiting for firmware/OS initialization
        DEFECTIVE  // Permanent failure lockout
    };

    /** @brief   Structure to track the lifecycle and health of a managed node. */
    struct ManagedNode {
        std::string name;
        Tier tier{Tier::FATAL};
        int retry_count{0};
        uint8_t current_state{255};  // 255 - Unconfigured/Unknown

        // Finite State Machine context
        RecoveryPhase phase{RecoveryPhase::NOMINAL};
        rclcpp::Time last_phase_change;
        bool is_defective{false};

        // Configuration parameters from fdir_policy.yaml
        double startup_time{0.0};
        double power_off_time{0.0};
        ResetMechanism reset_mechanism{ResetMechanism::LIFECYCLE_SERVICE};
        std::string reset_topic;

        // True once the component has reached ACTIVE. Distinguishes the initial
        // boot (not yet ACTIVE) from a failure of a running component.
        bool was_active{false};

        // Consecutive monitor cycles in which the component did not report its state
        int missed_polls{0};
    };

    // Configuration parameters from fdir_policy.yaml
    int service_timeout_ms_{50};  // [ms] Response deadline of a lifecycle service request
    int unresponsive_poll_limit_{3};  // Consecutive missed polls that declare a failure

    // Multi-threaded execution management.
    // The monitor timer runs in a MutuallyExclusive group, so monitor_loop() never
    // overlaps with itself. Lifecycle service clients run in a separate Reentrant
    // group, so their responses are processed while monitor_loop() waits for them.
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;

    // Communication interfaces
    rclcpp::Publisher<mona_msgs::msg::FdirState>::SharedPtr health_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contactor_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
    // Global Velocity Override (Secondary Safety Layer)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_override_pub_;

    // State tracking unordered_maps
    std::unordered_map<std::string, ManagedNode> managed_nodes_;
    std::unordered_map<std::string,
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients_;
    std::unordered_map<std::string,
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients_;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> relay_pubs_;

    // Core initialization and monitoring logic
    void init_managed_nodes();
    void monitor_loop();
    void handle_recovery_fsm(ManagedNode &node);
    static uint8_t failure_state(const ManagedNode &node);

    // Lifecycle interaction helpers
    uint8_t get_node_state(const std::string &node_name);
    bool change_node_state(const std::string &node_name, uint8_t transition_id);
};
}  // namespace mona_core

#endif  // MONA_CORE__FDIR_MANAGER_NODE_HPP_
