# Code Review Report (2026-05-04)

## Scope

Review target: changes introduced by merge `HEAD^1..HEAD` on branch `code-review-2026-05-04`.

Reviewed areas:
- `mona_control` (`twist_mux_node`)
- `mona_core` (`fdir_manager_node`)
- related documentation/config formatting updates

## Findings

### 1) High severity: potential crash after safety unblock in TwistMux

**File:** `src/mona_control/src/twist_mux_node.cpp`  
**Area:** `TwistMuxNode::control_loop()`

When safety is active, state is forced to `BLOCKED`. After unblock, if command timeout has not yet elapsed, the state router accepts only `MANUAL` and `AUTONOMOUS`. If the state remains `BLOCKED` in that cycle, code enters the fallback branch and throws `std::runtime_error`, causing process termination.

**Why this is risky**
- Can trigger unnecessary node restarts during normal recovery from emergency/protective stop.
- May create avoidable FDIR churn and transient loss of controllability.

**Suggested fix direction**
- Handle `BLOCKED` as an expected transitional state after unblock.
- Convert `BLOCKED` to `IDLE` (or a guarded safe default) before the routing block.
- Keep fatal path only for truly unreachable/invalid enum values.

### 2) Medium severity: startup race risk in FDIR manager timer initialization

**File:** `src/mona_core/src/fdir_manager_node.cpp`  
**Area:** `FdirManagerNode` constructor order

`monitor_timer_` is created before `init_managed_nodes()`. In a multi-threaded executor this may allow `monitor_loop()` to run while managed structures/clients are still being initialized.

**Why this is risky**
- Non-deterministic startup behavior.
- Potential access to partially initialized state in monitor loop.

**Suggested fix direction**
- Start timer only after `init_managed_nodes()` completes.
- Optionally gate `monitor_loop()` with an `is_initialized_` flag for defensive safety.

## Overall assessment

The change set improves readability and default initialization in several modules, but includes two behavioral risks that should be addressed before relying on this branch in safety-critical runs.
