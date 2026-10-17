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


#ifndef MONA_SAFETY__STOP_MONITOR_HPP_
#define MONA_SAFETY__STOP_MONITOR_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>

namespace mona_safety
{
/** @brief   Configuration of the time-monitored stop. */
struct StopMonitorConfig {
    double deceleration{1.0};  // [m/s^2]   Minimum planar deceleration of a controlled stop
    double angular_deceleration{1.0};  // [rad/s^2] Minimum yaw deceleration of a controlled stop
    double time_margin{0.5};  // [s]  Allowance for command latency, drive response, odometry delay
    double standstill_linear{0.05};  // [m/s]   Planar speed regarded as standstill
    double standstill_angular{0.05};  // [rad/s] Yaw rate regarded as standstill
    int debounce_samples{3};  // Consecutive moving samples that confirm a violation
};

/**
 * @brief   Time-monitored stop in the sense of SS1-t (IEC 61800-5-2), free of ROS
 *          dependencies so that the logic is unit-testable with synthetic time.
 *          On entry into a stop, the permitted stopping time is derived from the
 *          measured entry speed and the minimum deceleration, plus a fixed margin.
 *          Motion before this deadline is the expected braking. Motion after it,
 *          confirmed by debounce_samples consecutive samples, violates the stop.
 *          A non-finite speed sample is treated as motion (fail-safe).
 */
class StopMonitor {
public:
    using Clock = std::chrono::steady_clock;

    explicit StopMonitor(const StopMonitorConfig &config)
        : config_(config) {}

    /** @return  true if every value is finite and positive (debounce at least 1). */
    static bool is_valid(const StopMonitorConfig &config) {
        const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
        return positive(config.deceleration) && positive(config.angular_deceleration) &&
               positive(config.time_margin) && positive(config.standstill_linear) &&
               positive(config.standstill_angular) && config.debounce_samples >= 1;
    }

    /**
     * @brief   Starts the supervision of a stop.
     * @param   now           - Time of entry into the stop.
     * @param   linear_speed  - [m/s]   Measured planar speed at entry.
     * @param   angular_speed - [rad/s] Measured yaw rate at entry.
     */
    void begin(Clock::time_point now, double linear_speed, double angular_speed) {
        double stop_time = std::max(
            std::abs(linear_speed) / config_.deceleration,
            std::abs(angular_speed) / config_.angular_deceleration) + config_.time_margin;

        // A non-finite entry speed cannot justify any braking time: margin only
        if (!std::isfinite(stop_time)) {
            stop_time = config_.time_margin;
        }

        deadline_ = now + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(stop_time));
        moving_samples_ = 0;
        active_         = true;
    }

    /** @brief   Ends the supervision when the stop is released. */
    void end() {
        active_         = false;
        moving_samples_ = 0;
    }

    bool active() const {return active_;}

    /** @return  true if a stop is supervised and its permitted stopping time has elapsed. */
    bool deadline_expired(Clock::time_point now) const {return active_ && now >= deadline_;}

    /**
     * @brief   Evaluates one odometry sample.
     * @return  true if the sample confirms motion after the permitted stopping time.
     */
    bool update(Clock::time_point now, double linear_speed, double angular_speed) {
        if (!deadline_expired(now)) {
            moving_samples_ = 0;
            return false;
        }

        // Negated comparison: NaN fails every comparison and is therefore "moving"
        const bool moving = !(std::abs(linear_speed) <= config_.standstill_linear) ||
            !(std::abs(angular_speed) <= config_.standstill_angular);

        moving_samples_ = moving ? std::min(moving_samples_ + 1, config_.debounce_samples) : 0;
        return moving_samples_ >= config_.debounce_samples;
    }

private:
    StopMonitorConfig config_;
    Clock::time_point deadline_{};
    int moving_samples_{0};
    bool active_{false};
};
}  // namespace mona_safety

#endif  // MONA_SAFETY__STOP_MONITOR_HPP_
