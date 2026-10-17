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


#include "mona_core/fdir_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>

#include "lifecycle_msgs/msg/state.hpp"

// Colors for console output formatting
#define COLOR_BLUE      "\033[34m"
#define COLOR_YELLOW    "\033[1;33m"
#define COLOR_BOLD_RED  "\033[1;31m"
#define COLOR_RESET     "\033[0m"


namespace mona_core
{
/**
 * @brief   Construct a new FDIR Manager Node object.
 *          Initializes QoS profiles, ROS 2 communication interfaces, and
 *          sets up the deterministic high-frequency safety monitoring loop.
 * @param   options - ROS 2 Node options
 */
FdirManagerNode::FdirManagerNode(const rclcpp::NodeOptions &options)
    : Node("mona_fdir_manager", options) {
    // Initialize Reentrant group to prevent deadlocks during blocking service calls
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // MutuallyExclusive group: a delayed monitor cycle is skipped, never run concurrently
    timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Initialize deterministic safety monitoring loop (10 Hz = 100 ms)
    monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(MONITOR_PERIOD_MS),
        std::bind(&FdirManagerNode::monitor_loop, this),
        timer_group_);

    // QoS: Reliable + Transient Local for safety-critical state broadcast
    rclcpp::QoS qos_profile(10);
    qos_profile.reliable();
    qos_profile.transient_local();

    health_state_pub_ = this->create_publisher<mona_msgs::msg::FdirState>(
        "system/health_state", qos_profile);

    // Initialize Hardware Watchdog Heartbeat
    // Using Best Effort QoS to minimize latency; dropped packets are treated as faults by PLC.
    rclcpp::QoS heartbeat_qos(10);
    heartbeat_qos.best_effort();

    heartbeat_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "hardware/fdir_heartbeat", heartbeat_qos);

    // Emergency Hardware Actuation
    contactor_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "hardware/contactors", 10);

    cmd_vel_override_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "hardware/motor_cmd", 10);

    // Parse configurations and provision dynamic interfaces
    init_managed_nodes();

    RCLCPP_INFO(this->get_logger(), "FDIR Manager initialized.");
}

FdirManagerNode::~FdirManagerNode() {
    // CRITICAL FAULT MITIGATION: Explicitly cancel the high-frequency safety timer.
    // Prevents the ROS 2 MultiThreadedExecutor from invoking monitor_loop() on a
    // deallocated 'this' pointer during abrupt component unloads (Chaos Engineering).
    if (monitor_timer_) {
        monitor_timer_->cancel();
        monitor_timer_.reset();
    }
}

/**
 * @brief   Parses component configurations and initializes ROS 2 communication interfaces.
 *          Dynamically provisions Lifecycle clients and hardware relay publishers based on the
 *          defined reset mechanism. Implements strict configuration validation and fail-safe
 *          boot halting on invalid parameters. Adheres to the "Declare First" pattern.
 */
void FdirManagerNode::init_managed_nodes() {
    // PARAMETER DECLARATION
    this->declare_parameter("recovery_power_off_time", 3.0);
    this->declare_parameter("service_timeout_ms", 50);
    this->declare_parameter("unresponsive_poll_limit", 3);
    this->declare_parameter("tracked_components", std::vector<std::string>{});

    // Response deadline: one state request must fit into one monitor cycle
    const int64_t service_timeout_ms = this->get_parameter("service_timeout_ms").as_int();
    if (service_timeout_ms <= 0 || service_timeout_ms >= MONITOR_PERIOD_MS) {
        RCLCPP_FATAL(
            this->get_logger(),
            "[FDIR] CONFIGURATION FAULT: service_timeout_ms = %ld is outside (0, %d).",
            service_timeout_ms, MONITOR_PERIOD_MS);
        throw std::runtime_error("Invalid service_timeout_ms in FDIR policy configuration.");
    }
    service_timeout_ms_ = static_cast<int>(service_timeout_ms);

    // Failure declaration: bounded to keep the detection time within 1 s
    const int64_t poll_limit = this->get_parameter("unresponsive_poll_limit").as_int();
    if (poll_limit < 1 || poll_limit > 10) {
        RCLCPP_FATAL(
            this->get_logger(),
            "[FDIR] CONFIGURATION FAULT: unresponsive_poll_limit = %ld is outside [1, 10].",
            poll_limit);
        throw std::runtime_error("Invalid unresponsive_poll_limit in FDIR policy configuration.");
    }
    unresponsive_poll_limit_ = static_cast<int>(poll_limit);

    // Fetch the component list to determine dynamic parameter prefixes
    auto tracked_components = this->get_parameter("tracked_components").as_string_array();

    // Critical validation: Ensure tracked components are provided before proceeding
    if (tracked_components.empty()) {
        RCLCPP_FATAL(
            this->get_logger(),
            "[FDIR] CONFIGURATION FAULT: No tracked_components found!");
        throw std::runtime_error("Missing tracked_components in FDIR policy configuration.");
    }

    // Pre-declare all dynamic component parameters
    for (const auto &comp_key : tracked_components) {
        std::string prefix = "components." + comp_key + ".";
        this->declare_parameter(prefix + "enabled", false);
        this->declare_parameter(prefix + "node_name", "");
        this->declare_parameter(prefix + "tier", "");
        this->declare_parameter(prefix + "reset_mechanism", "");
        this->declare_parameter(prefix + "startup_time", 5.0);
        this->declare_parameter(prefix + "reset_topic", "");
    }

    // PARAMETER FETCHING & STRICT VALIDATION
    double global_power_off = this->get_parameter("recovery_power_off_time").as_double();

    // Parse each component's configuration dynamically
    for (const auto &comp_key : tracked_components) {
        std::string prefix = "components." + comp_key + ".";

        // Validate if component monitoring is explicitly enabled
        if (!this->get_parameter(prefix + "enabled").as_bool()) {
            continue;
        }

        // Fundamental attributes
        std::string node_name = this->get_parameter(prefix + "node_name").as_string();

        if (node_name.empty()) {
            RCLCPP_FATAL(
                this->get_logger(), "[FDIR] CONFIGURATION FAULT: Empty node_name for key '%s'.",
                comp_key.c_str());
            throw std::runtime_error("Invalid FDIR configuration: Empty node_name.");
        }

        // Tier evaluation: Invalid tier halts system boot.
        std::string tier_str = this->get_parameter(prefix + "tier").as_string();
        Tier        tier{Tier::FATAL};  // Most restrictive tier until parsed

        if (tier_str == "FATAL") {
            tier = Tier::FATAL;
        } else if (tier_str == "PRIMARY") {
            tier = Tier::PRIMARY;
        } else if (tier_str == "AUXILIARY") {
            tier = Tier::AUXILIARY;
        } else {
            RCLCPP_FATAL(
                this->get_logger(),
                "[FDIR] CONFIGURATION FAULT: Unknown tier '%s' for '%s'. "
                "Validation failed.",
                tier_str.c_str(), node_name.c_str());
            throw std::runtime_error("Invalid safety tier configuration.");
        }

        // Reset Mechanism evaluation.
        std::string mech_str = this->get_parameter(prefix + "reset_mechanism").as_string();
        // Overwritten below, or the configuration is rejected
        ResetMechanism mechanism{ResetMechanism::LIFECYCLE_SERVICE};

        if (mech_str == "hardware_relay") {
            mechanism = ResetMechanism::HARDWARE_RELAY;
        } else if (mech_str == "lifecycle_service") {
            mechanism = ResetMechanism::LIFECYCLE_SERVICE;
        } else {
            RCLCPP_FATAL(
                this->get_logger(),
                "[FDIR] CONFIGURATION FAULT: Unknown reset_mechanism '%s' for '%s'. "
                "Validation failed.",
                mech_str.c_str(), node_name.c_str());
            throw std::runtime_error("Invalid reset_mechanism configuration.");
        }

        // Timing and hardware endpoints
        double      startup_time = this->get_parameter(prefix + "startup_time").as_double();
        std::string reset_topic  = this->get_parameter(prefix + "reset_topic").as_string();

        // Cross-validation for hardware relays
        if (mechanism == ResetMechanism::HARDWARE_RELAY && reset_topic.empty()) {
            RCLCPP_FATAL(
                this->get_logger(),
                "[FDIR] CONFIGURATION FAULT: 'hardware_relay' specified for '%s' "
                "but 'reset_topic' is empty.",
                node_name.c_str());
            throw std::runtime_error("Missing reset_topic for hardware relay mechanism.");
        }

        // INFRASTRUCTURE PROVISIONING
        // 1. Initialize ManagedNode structure
        managed_nodes_[node_name] = {
            node_name, tier, 0,
            lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
            RecoveryPhase::NOMINAL,
            this->get_clock()->now(),
            false,
            startup_time,
            global_power_off,  // Applies globally defined power-off timing
            mechanism,
            reset_topic,
            false,  // was_active: the component has not reached ACTIVE yet
            0  // missed_polls
        };

        // 2. Provision Communication Clients (Action interfaces)
        get_state_clients_[node_name] = this->create_client<lifecycle_msgs::srv::GetState>(
            node_name + "/get_state", rmw_qos_profile_services_default, callback_group_);

        change_state_clients_[node_name] = this->create_client<lifecycle_msgs::srv::ChangeState>(
            node_name + "/change_state", rmw_qos_profile_services_default, callback_group_);

        // 3. Provision Hardware Relay Publishers (Only if required)
        if (mechanism == ResetMechanism::HARDWARE_RELAY) {
            relay_pubs_[node_name] = this->create_publisher<std_msgs::msg::Bool>(
                reset_topic, rclcpp::QoS(1).transient_local());
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[FDIR] Monitoring Component: %s (Tier: %s, Reset: %s)",
            node_name.c_str(), tier_str.c_str(), mech_str.c_str());
    }
}

/**
 * @brief   Queries the current lifecycle state of a specified managed component.
 *          Implements strict timeouts to prevent the FDIR thread from deadlocking if
 *          a target node experiences a Byzantine fault or infinite loop.
 * @param   node_name - Exact name of the target lifecycle node.
 * @return  uint8_t   - State ID. Returns PRIMARY_STATE_UNKNOWN on timeout or fault.
 */
uint8_t FdirManagerNode::get_node_state(const std::string &node_name) {
    try {
        auto client = get_state_clients_.at(node_name);

        // Non-blocking availability check to prevent Real-Time Executor starvation
        if (!client->service_is_ready()) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "[FDIR] OFFLINE: get_state service for '%s' "
                "is not ready (Expected during reset).",
                node_name.c_str());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future  = client->async_send_request(request);

        // Block with a strict deterministic timeout
        if (future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) ==
            std::future_status::ready)
        {
            // If the target node was destroyed while we were waiting,
            // future.get() will throw a std::future_error.
            auto response = future.get();
            if (response) {
                return response->current_state.id;
            } else {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "[FDIR] SECURITY FAULT: get_state request aborted (node '%s' crashed).",
                    node_name.c_str());
                return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            }
        }

        // Deadline missed: a late response is discarded instead of being kept pending.
        // A single missed deadline is not a failure; monitor_loop() debounces it.
        client->remove_pending_request(future);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[FDIR] TIMEOUT: Node '%s' did not report its state within %d ms.",
            node_name.c_str(), service_timeout_ms_);
    } catch (const std::future_error &e) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "[FDIR] BROKEN PROMISE: Target node '%s' was destroyed during get_state request.",
            node_name.c_str());
    } catch (const std::out_of_range &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "[FDIR] MEMORY FAULT: Attempted to query state for unmanaged node '%s'.",
            node_name.c_str());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "[FDIR] SYSTEM FAULT in get_state for '%s': %s",
            node_name.c_str(), e.what());
    }

    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
}

/**
 * @brief   Requests a deterministic Lifecycle state transition for a specified node.
 *          Bypasses unresponsive nodes to prevent the FDIR orchestrator from stalling.
 * @param   node_name     - Exact name of the target lifecycle node.
 * @param   transition_id - Transition ID (e.g., TRANSITION_ACTIVATE).
 * @return  true if the transition was successfully acknowledged, false otherwise.
 */
bool FdirManagerNode::change_node_state(const std::string &node_name, uint8_t transition_id) {
    try {
        auto client = change_state_clients_.at(node_name);

        // Non-blocking availability check
        if (!client->service_is_ready()) {
            RCLCPP_DEBUG(
                this->get_logger(),
                "[FDIR] OFFLINE: change_state service for '%s' "
                "is not ready (Expected during reset).",
                node_name.c_str());
            return false;
        }

        auto request           = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;

        auto future = client->async_send_request(request);

        // Strict deterministic execution bound
        if (future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) ==
            std::future_status::ready)
        {
            auto response = future.get();
            if (response) {
                return response->success;
            } else {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[FDIR] SECURITY FAULT: change_state request aborted (node '%s' crashed).",
                    node_name.c_str());
                return false;
            }
        }

        client->remove_pending_request(future);
        RCLCPP_ERROR(
            this->get_logger(),
            "[FDIR] SECURITY FAULT: Transition request to '%s' timed out after %d ms.",
            node_name.c_str(), service_timeout_ms_);
    } catch (const std::future_error &e) {
        RCLCPP_ERROR(
            this->get_logger(),
            "[FDIR] BROKEN PROMISE: Target node '%s' was destroyed during change_state request.",
            node_name.c_str());
    } catch (const std::out_of_range &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "[FDIR] MEMORY FAULT: Attempted state transition for unmanaged node '%s'.",
            node_name.c_str());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "[FDIR] SYSTEM FAULT in change_state for '%s': %s",
            node_name.c_str(), e.what());
    }

    return false;
}

/**
 * @brief   Core 10 Hz control loop evaluating system health and orchestrating recovery.
 *          Implements a pessimistic start pattern and escalates failures based on tier levels.
 *          This deterministic cycle ensures high-frequency monitoring and autonomous
 *          restoration of lifecycle-managed components.
 */
void FdirManagerNode::monitor_loop() {
    // Pessimistic initialization: assume startup phase until proven otherwise
    uint8_t global_state = mona_msgs::msg::FdirState::STATE_SYSTEM_STARTUP;

    // Assumed truth pattern: Assume all nodes are active, falsify if any is not
    bool all_active = true;

    for (auto & [name, node] : managed_nodes_) {
        // Bypass evaluation if the component is designated
        // as permanently defective (Safety Lockout)
        if (node.is_defective) {
            all_active   = false;
            global_state = std::max(
                global_state,
                mona_msgs::msg::FdirState::STATE_EMERGENCY);
            continue;
        }

        uint8_t state = get_node_state(name);

        // DEBOUNCE: a component that does not report its state is declared failed only
        // after unresponsive_poll_limit_ consecutive missed polls. Until then, its last
        // confirmed state is kept and no lifecycle transition is sent. Recovery in
        // progress is not debounced.
        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN) {
            node.missed_polls = 0;
        } else if (node.phase == RecoveryPhase::NOMINAL) {
            node.missed_polls++;
            if (node.missed_polls < unresponsive_poll_limit_) {
                if (node.current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    all_active   = false;
                    global_state = std::max(global_state, failure_state(node));
                }
                continue;
            }
        }
        node.current_state = state;

        // ANTI-ZOMBIE / SECURITY BREACH DETECTION
        if ((node.is_defective) && (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)) {
            RCLCPP_FATAL(
                this->get_logger(),
                "[FDIR] SECURITY BREACH: Locked-out defective component '%s' "
                "illegally reported ACTIVE state! "
                "System integrity compromised. Engaging hard lock.",
                name.c_str());

            // Latch the entire system into a non-recoverable FATAL state
            global_state = mona_msgs::msg::FdirState::STATE_EMERGENCY;
            all_active   = false;
            continue;
        }

        switch (state) {
            case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
                if (node.phase == RecoveryPhase::WAIT_BOOT) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        COLOR_BLUE "[FDIR] '%s' stabilized and ACTIVE." COLOR_RESET,
                        name.c_str());
                }
                node.retry_count = 0;
                node.phase       = RecoveryPhase::NOMINAL;
                node.was_active  = true;
                break;

            case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
                all_active   = false;
                global_state = std::max(global_state, failure_state(node));

                // Component requires initialization
                change_node_state(name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[FDIR] Component '%s' is UNCONFIGURED. "
                    "Triggering 'configure' transition...",
                    name.c_str());
                break;

            case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
                all_active   = false;
                global_state = std::max(global_state, failure_state(node));

                // Component is configured but dormant. Initiating activation sequence.
                change_node_state(name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[FDIR] Component '%s' is INACTIVE. "
                    "Triggering 'activate' transition...",
                    name.c_str());
                break;

            default:
                // UNKNOWN, ERROR, or finalizing state -> Component is considered failed
                all_active = false;

                // EDGE-TRIGGERED LOGGING: Only log the fault warning at the exact moment
                // it transitions out of NOMINAL operation to prevent terminal spam during recovery.
                if (node.phase == RecoveryPhase::NOMINAL) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "[FDIR] Failure detected in '%s' (State ID: %d). "
                        "Initiating recovery sequence...",
                        name.c_str(), state);
                }

                handle_recovery_fsm(node);

                // If the recovery FSM just declared the node permanently defective,
                // instantly escalate to EMERGENCY in the current tick.
                if (node.is_defective) {
                    global_state = std::max(
                        global_state,
                        mona_msgs::msg::FdirState::STATE_EMERGENCY);
                } else {
                    // Escalate according to the tier of the failing component while
                    // recovery is in progress (STARTUP before its first activation).
                    global_state = std::max(global_state, failure_state(node));
                }
                break;
        }
    }

    // Elevate status if the full software stack is operational
    if (all_active) {
        global_state = mona_msgs::msg::FdirState::STATE_SOFTWARE_OK;
    }

    // Broadcast current FDIR Health State to the telemetry interfaces
    auto msg               = mona_msgs::msg::FdirState();
    msg.header.stamp       = this->get_clock()->now();
    msg.current_state      = global_state;
    msg.diagnostic_message = all_active ?
        "Nominal operation." : "One or more components are recovering or failed.";
    health_state_pub_->publish(msg);

    // System State Logging & Hardware Control Logic
    auto contactor_msg = std_msgs::msg::Bool();
    auto zero_twist    = geometry_msgs::msg::Twist();

    switch (global_state) {
        case mona_msgs::msg::FdirState::STATE_EMERGENCY:
            contactor_msg.data = false;  // Interlock: Cut power to drivetrain
            contactor_pub_->publish(contactor_msg);
            cmd_vel_override_pub_->publish(zero_twist);

            RCLCPP_FATAL_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                COLOR_BOLD_RED "EMERGENCY: HARDWARE POWER CUTOFF AND "
                "ZERO VELOCITY OVERRIDE TRIGGERED!" COLOR_RESET);
            break;

        case mona_msgs::msg::FdirState::STATE_PROTECTIVE_STOP:
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                COLOR_YELLOW "PROTECTIVE STOP: Software motion halt active." COLOR_RESET);
            break;

        case mona_msgs::msg::FdirState::STATE_DEGRADED:
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                COLOR_YELLOW "DEGRADED MODE: Auxiliary component offline." COLOR_RESET);
            break;

        case mona_msgs::msg::FdirState::STATE_SYSTEM_STARTUP:
            RCLCPP_INFO_ONCE(
                this->get_logger(),
                "Initializing components...");
            break;

        case mona_msgs::msg::FdirState::STATE_SOFTWARE_OK:
            // Terminal feedback for successful initialization and stabilization
            RCLCPP_INFO_ONCE(
                this->get_logger(),
                COLOR_BLUE "SYSTEM READY. ALL MODULES OPERATIONAL." COLOR_RESET);
            break;

        default:
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(), 1000, "INVALID SYSTEM STATE!");
            break;
    }

    // ==============================================================================
    // HARDWARE WATCHDOG (DEAD MAN'S SWITCH) - MISSION CRITICAL
    // ==============================================================================
    // Emit a continuous high-frequency pulse (10 Hz) to the low-level PLC/HAL.
    // If the FDIR node crashes, freezes, or the OS panics, this signal stops.
    // The hardware PLC must physically open the safety contactors if no pulse
    // is received within a defined latency window (e.g., 200 ms).
    auto heartbeat_msg = std_msgs::msg::Bool();
    heartbeat_msg.data = true;
    heartbeat_pub_->publish(heartbeat_msg);
}

/**
 * @brief   Maps a component that is not ACTIVE to the global health state it requires.
 *          Before the component has reached ACTIVE for the first time, the system is
 *          booting (STARTUP). Afterwards, the state follows the tier of the component:
 *          a failing FATAL component triggers EMERGENCY immediately, without waiting
 *          for the recovery attempts to be exhausted.
 * @param   node - Managed component that is not ACTIVE.
 * @return  uint8_t - FdirState value required by the component's condition.
 */
uint8_t FdirManagerNode::failure_state(const ManagedNode &node) {
    if (!node.was_active) {
        return mona_msgs::msg::FdirState::STATE_SYSTEM_STARTUP;
    }

    switch (node.tier) {
        case Tier::FATAL:
            return mona_msgs::msg::FdirState::STATE_EMERGENCY;
        case Tier::PRIMARY:
            return mona_msgs::msg::FdirState::STATE_PROTECTIVE_STOP;
        case Tier::AUXILIARY:
            return mona_msgs::msg::FdirState::STATE_DEGRADED;
        default:  // Out-of-range tier value (memory corruption): most restrictive state
            return mona_msgs::msg::FdirState::STATE_EMERGENCY;
    }
}

/**
 * @brief   Executes deterministic recovery sequences utilizing a non-blocking FSM.
 *          Orchestrates hardware/software resets with precise timing and visual telemetry.
 *          Action execution is prioritized over logging to ensure minimal safety latency.
 * @param   node - Reference to the managed component's state data.
 */
void FdirManagerNode::handle_recovery_fsm(ManagedNode &node) {
    rclcpp::Time now          = this->get_clock()->now();
    double       elapsed_time = (now - node.last_phase_change).seconds();

    switch (node.phase) {
        case RecoveryPhase::NOMINAL:
            node.retry_count++;
            if (node.retry_count > MAX_RECOVERY_RETRIES) {
                node.is_defective = true;
                node.phase        = RecoveryPhase::DEFECTIVE;

                RCLCPP_FATAL(
                    this->get_logger(),
                    "\n"
                    "=================================================================\n"
                    "[FDIR] TERMINAL FAULT: '%s' is physically unresponsive.\n"
                    "[FDIR] Recovery exhausted (%d attempts). Component LOCKED OUT.\n"
                    "[FDIR] SYSTEM HALTED. REQUIRES FULL MANUAL RESTART.\n"
                    "=================================================================",
                    node.name.c_str(), MAX_RECOVERY_RETRIES);
                return;
            } else {
                node.phase = RecoveryPhase::POWER_OFF;
            }
            break;

        case RecoveryPhase::POWER_OFF:
            // Actuate immediately based on defined mechanism to ensure minimal safety latency
            if (node.reset_mechanism == ResetMechanism::HARDWARE_RELAY) {
                auto msg = std_msgs::msg::Bool();
                msg.data = false;
                relay_pubs_[node.name]->publish(msg);

                RCLCPP_WARN(
                    this->get_logger(),
                    "[FDIR] Initiating Hard Power Cycle for '%s'. RELAY OPENED.",
                    node.name.c_str());
            } else if (node.reset_mechanism == ResetMechanism::LIFECYCLE_SERVICE) {
                // A lifecycle transition is sent only if it is valid for the component's
                // actual state, which is queried again before the reset.
                const uint8_t actual_state = get_node_state(node.name);

                if (actual_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                    // The component reports ACTIVE again: the failure condition is gone
                    node.phase             = RecoveryPhase::NOMINAL;
                    node.last_phase_change = now;

                    RCLCPP_WARN(
                        this->get_logger(),
                        "[FDIR] '%s' reports ACTIVE again. Recovery cancelled.",
                        node.name.c_str());
                    return;
                }

                if (actual_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                    change_node_state(
                        node.name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "[FDIR] Initiating Software Lifecycle Reset for '%s'.",
                        node.name.c_str());
                } else {
                    // UNCONFIGURED needs no reset; UNKNOWN, ERRORPROCESSING and
                    // FINALIZED accept no externally requested transition.
                    RCLCPP_INFO(
                        this->get_logger(),
                        "[FDIR] No lifecycle reset is valid for '%s' in state %d. "
                        "Waiting for the component.",
                        node.name.c_str(), actual_state);
                }
            } else {
                // Security/Integrity fallback: Catch uninitialized or corrupted mechanism states
                node.is_defective = true;
                node.phase        = RecoveryPhase::DEFECTIVE;

                RCLCPP_FATAL(
                    this->get_logger(),
                    "[FDIR] INTEGRITY FAULT: Unknown reset mechanism for '%s'. Engaging lockout.",
                    node.name.c_str());
                return;
            }

            // Update FSM state to wait for capacitor discharge or software teardown
            node.phase             = RecoveryPhase::WAIT_DISCHARGE;
            node.last_phase_change = now;
            break;

        case RecoveryPhase::WAIT_DISCHARGE:
            // State Edge Logging: Emit a transitional notification strictly on the
            // initial control cycle. By bounding the elapsed time to the first
            // execution tick (~100ms period), we ensure deterministic observability
            // of the FSM progression without inducing console spam.
            if (elapsed_time <= 0.15) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[FDIR] Hardware discharging phase initiated for '%s'. "
                    "Holding for %.1f seconds...",
                    node.name.c_str(), node.power_off_time);
            }

            if (elapsed_time >= node.power_off_time) {
                // Transition to POWER_ON phase after the guaranteed discharge duration has elapsed
                node.phase = RecoveryPhase::POWER_ON;
            }
            break;

        case RecoveryPhase::POWER_ON:
            // Actuate immediately to restore nominal state
            if (node.reset_mechanism == ResetMechanism::HARDWARE_RELAY) {
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                relay_pubs_[node.name]->publish(msg);

                RCLCPP_INFO(
                    this->get_logger(),
                    "[FDIR] Power restored to '%s'. RELAY CLOSED.",
                    node.name.c_str());
            } else if (node.reset_mechanism == ResetMechanism::LIFECYCLE_SERVICE) {
                // Software mechanism requires no explicit power-on action, proceed to boot logging
                RCLCPP_INFO(
                    this->get_logger(),
                    "[FDIR] Software reset complete for '%s'. Preparing to boot.",
                    node.name.c_str());
            } else {
                // Security/Integrity fallback during power restoration
                node.is_defective = true;
                node.phase        = RecoveryPhase::DEFECTIVE;

                RCLCPP_FATAL(
                    this->get_logger(),
                    "[FDIR] INTEGRITY FAULT: Unknown reset mechanism during POWER_ON for '%s'.",
                    node.name.c_str());
                return;
            }

            // Update FSM state to wait for OS/Firmware initialization
            node.phase             = RecoveryPhase::WAIT_BOOT;
            node.last_phase_change = now;
            break;

        case RecoveryPhase::WAIT_BOOT:
            // State Edge Logging: Acknowledge the start of
            // the boot phase on the initial control cycle.
            if (elapsed_time <= 0.15) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[FDIR] Boot sequence initiated for '%s'. "
                    "Waiting %.1f seconds for initialization...",
                    node.name.c_str(), node.startup_time);
            }

            if (elapsed_time >= node.startup_time) {
                // Boot duration exhausted. Transition back to NOMINAL
                // to allow the primary monitor_loop to evaluate
                // if the node successfully reached the ACTIVE state.
                node.phase             = RecoveryPhase::NOMINAL;
                node.last_phase_change = now;

                RCLCPP_INFO(
                    this->get_logger(),
                    "[FDIR] Boot sequence completed for '%s'. Evaluating component health...",
                    node.name.c_str());
            }
            break;

        case RecoveryPhase::DEFECTIVE:
            // Node is permanently locked out due to critical failure. No further action taken.
            break;

        default:
            // Ultimate safety catch-all for corrupted memory or invalid enum states
            node.is_defective = true;
            node.phase        = RecoveryPhase::DEFECTIVE;

            RCLCPP_FATAL(
                this->get_logger(),
                "[FDIR] CRITICAL FAULT: Invalid FSM phase detected for '%s'. Engaging Lockout.",
                node.name.c_str());
            break;
    }
}
}  // namespace mona_core


// Register the component within the ROS 2 infrastructure
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mona_core::FdirManagerNode)
