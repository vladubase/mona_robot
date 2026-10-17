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
#include <chrono>
#include <limits>

#include "mona_safety/stop_monitor.hpp"

using namespace std::chrono_literals;
using mona_safety::StopMonitor;
using mona_safety::StopMonitorConfig;

// Synthetic time: the tests run deterministically, independent of host load
static const StopMonitor::Clock::time_point T0{};

// Test 1: Braking within the permitted stopping time is not a violation
TEST(StopMonitorTest, BrakingWithinPermittedTimeIsAccepted) {
    StopMonitor monitor{StopMonitorConfig{}};

    // Entry at 1.0 m/s, 1.0 m/s^2, margin 0.5 s: permitted stopping time 1.5 s
    monitor.begin(T0, 1.0, 0.0);

    EXPECT_FALSE(monitor.update(T0 + 500ms, 0.5, 0.0));
    EXPECT_FALSE(monitor.update(T0 + 1400ms, 0.2, 0.0));
    EXPECT_FALSE(monitor.update(T0 + 1450ms, 0.2, 0.0));
    EXPECT_FALSE(monitor.update(T0 + 1490ms, 0.2, 0.0));
}

// Test 2: Motion after the permitted stopping time is a violation after debouncing
TEST(StopMonitorTest, MotionAfterDeadlineIsViolationAfterDebounce) {
    StopMonitor monitor{StopMonitorConfig{}};
    monitor.begin(T0, 1.0, 0.0);

    EXPECT_FALSE(monitor.update(T0 + 1500ms, 0.2, 0.0));
    EXPECT_FALSE(monitor.update(T0 + 1520ms, 0.2, 0.0));
    EXPECT_TRUE(monitor.update(T0 + 1540ms, 0.2, 0.0));
}

// Test 3: A single moving sample after the deadline is filtered out (noise)
TEST(StopMonitorTest, SingleMovingSampleIsFiltered) {
    StopMonitor monitor{StopMonitorConfig{}};
    monitor.begin(T0, 0.0, 0.0);

    EXPECT_FALSE(monitor.update(T0 + 600ms, 0.2, 0.0));
    EXPECT_FALSE(monitor.update(T0 + 620ms, 0.0, 0.0));
    EXPECT_FALSE(monitor.update(T0 + 640ms, 0.2, 0.0));
    EXPECT_FALSE(monitor.update(T0 + 660ms, 0.0, 0.0));
}

// Test 4: Rotation is supervised as well as translation
TEST(StopMonitorTest, RotationAfterDeadlineIsViolation) {
    StopMonitor monitor{StopMonitorConfig{}};

    // Entry at 0.8 rad/s, 1.0 rad/s^2: permitted stopping time 0.8 + 0.5 = 1.3 s
    monitor.begin(T0, 0.0, 0.8);

    EXPECT_FALSE(monitor.update(T0 + 1200ms, 0.0, 0.3));
    EXPECT_FALSE(monitor.update(T0 + 1300ms, 0.0, 0.3));
    EXPECT_FALSE(monitor.update(T0 + 1320ms, 0.0, -0.3));
    EXPECT_TRUE(monitor.update(T0 + 1340ms, 0.0, 0.3));
}

// Test 5: A non-finite speed sample is treated as motion (fail-safe)
TEST(StopMonitorTest, NonFiniteSampleIsTreatedAsMotion) {
    StopMonitor monitor{StopMonitorConfig{}};
    monitor.begin(T0, 0.0, 0.0);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(monitor.update(T0 + 600ms, nan, 0.0));
    EXPECT_FALSE(monitor.update(T0 + 620ms, nan, 0.0));
    EXPECT_TRUE(monitor.update(T0 + 640ms, nan, 0.0));
}

// Test 6: A non-finite entry speed permits only the margin as stopping time
TEST(StopMonitorTest, NonFiniteEntrySpeedPermitsMarginOnly) {
    StopMonitor monitor{StopMonitorConfig{}};
    monitor.begin(T0, std::numeric_limits<double>::infinity(), 0.0);

    EXPECT_FALSE(monitor.deadline_expired(T0 + 499ms));
    EXPECT_TRUE(monitor.deadline_expired(T0 + 500ms));
}

// Test 7: Without an active stop, no sample is a violation
TEST(StopMonitorTest, InactiveMonitorReportsNoViolation) {
    StopMonitor monitor{StopMonitorConfig{}};
    monitor.begin(T0, 0.0, 0.0);
    monitor.end();

    EXPECT_FALSE(monitor.active());
    EXPECT_FALSE(monitor.update(T0 + 10s, 1.0, 1.0));
    EXPECT_FALSE(monitor.update(T0 + 11s, 1.0, 1.0));
    EXPECT_FALSE(monitor.update(T0 + 12s, 1.0, 1.0));
}

// Test 8: Invalid configurations are rejected
TEST(StopMonitorTest, RejectsInvalidConfiguration) {
    EXPECT_TRUE(StopMonitor::is_valid(StopMonitorConfig{}));

    StopMonitorConfig zero_deceleration;
    zero_deceleration.deceleration = 0.0;
    EXPECT_FALSE(StopMonitor::is_valid(zero_deceleration));

    StopMonitorConfig nan_margin;
    nan_margin.time_margin = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(StopMonitor::is_valid(nan_margin));

    StopMonitorConfig no_debounce;
    no_debounce.debounce_samples = 0;
    EXPECT_FALSE(StopMonitor::is_valid(no_debounce));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
