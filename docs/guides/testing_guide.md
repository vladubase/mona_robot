# Testing and Validation Guide

> **Continuous Verification**
> 
> System reliability in the MONA architecture is guaranteed through a rigorous, multi-layered testing pipeline. This guide outlines how to execute tests locally, understand the CI pipeline, and write new unit tests for C++ and Python components.

## 1. Automated CI Pipeline (Local Testing)

Before opening a Pull Request, every developer is required to run the local Continuous Integration script. This script acts as a gatekeeper, running all static analyzers and unit tests within the isolated Docker environment.

**Execution variant a:**
```bash
./scripts/format_code.bash
./scripts/run_ci_checks.bash
```
**Execution variant b:**
```bash
make format
make ci
```

> [!NOTE]
> This script automatically executes `colcon test`, evaluates linter compliance, generates a line-coverage summary, and fails the run when the configured coverage threshold is not met.

---

## 2. C++ Unit Testing (GTest / Ament)

All C++ nodes and libraries must be covered by Google Test (`gtest`).

### Core Testing Patterns

Due to the safety-critical nature of MONA, we enforce testing of ROS 2 Node Lifecycles. Every component must have an instantiation test to ensure it handles memory correctly during initialization and shutdown.

**Example: Lifecycle Instantiation Test (`test_lifecycle_instantiation.cpp`)** This test verifies that a Managed Node can be successfully instantiated and destroyed without memory leaks or segfaults.
```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "mona_safety/safety_node.hpp"

TEST(SafetyNodeTest, Instantiation) {
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<mona_safety::SafetyNode>(options);
  
  EXPECT_EQ(node->get_name(), std::string("safety_node"));
  rclcpp::shutdown();
}
```

### Running Specific C++ Tests

To run tests for a single package and view the console output (useful for debugging):
```bash
colcon test --packages-select mona_safety --event-handlers console_direct+
```

---

## 3. FDIR and Safety Lifecycle Testing

The core FDIR Manager and Safety logic have been migrated to deterministic C++ Lifecycle components. Testing these modules requires verifying their Finite State Machine (FSM) transitions and hardware overrides (e.g., Zero Velocity Override).

To execute tests specifically for the safety core utilities with verbose output:
```bash
colcon test --packages-select mona_core mona_safety --event-handlers console_direct+
```

---

## 4. Static Analysis and Linters

MONA adheres strictly to ROS 2 style guidelines. Linters are automatically executed during the `colcon test` phase.
- **ament_uncrustify:** Enforces C++ formatting rules (configured via `configs/uncrustify.cfg`).
- **ament_cppcheck:** Detects undefined behavior, memory leaks, and uninitialized variables in C++.
- **ament_flake8:** Enforces PEP 8 compliance for all Python scripts.

If `uncrustify` fails, you can automatically format your C++ code by running:
```bash
ament_uncrustify --reformat src/
```

---

## 5. Code Coverage (Codecov)

The CI pipeline compiles the workspace with GCC coverage instrumentation (`--coverage`). When `lcov` is available, `run_ci_checks.bash` writes `build/coverage.info` and prints the standard `lcov --list` report. GitHub Actions uploads this report to Codecov on every Pull Request.

For local containers or older images where `lcov` is not installed, the script falls back to `gcov` and reports line coverage for the production C++ components:
- `mona_control::TwistMuxNode`
- `mona_safety::SafetyNode`
- `mona_perception::LidarMergerNode`
- `mona_core::FdirManagerNode`

The default local line-coverage gate is `60%`. A lower result fails CI after tests complete, so coverage regressions are visible even when all GTest suites pass.

To raise or lower the local gate while already inside the dev container:
```bash
COVERAGE_MIN_LINES=70 ./scripts/run_ci_checks.bash
```

Current verified local baseline:
```text
mona_control      74.86%
mona_safety       77.45%
mona_perception   80.00%
mona_core         49.83%
TOTAL             67.28%
```
