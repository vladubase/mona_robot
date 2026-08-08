# TODO: Potential Problems From Code Review (2026-05-04)

## Priority Tasks

- [x] **P0 / High** Fix `TwistMuxNode::control_loop()` state handling after safety unblock.  
      Ensure `BLOCKED` is treated as a valid transitional state and does not trigger fatal exception during normal recovery.

- [x] **P1 / Medium** Reorder FDIR initialization in `FdirManagerNode`.  
      Move `monitor_timer_` creation to after `init_managed_nodes()` to avoid startup race in multi-threaded execution.

## Validation Tasks

- [x] Add or update test scenario: `BLOCKED -> unblocked -> stable IDLE/MANUAL/AUTONOMOUS` without process crash.
- [ ] Add startup robustness test for FDIR manager under multi-threaded executor.
- [ ] Run lifecycle transition checks for emergency stop and recovery path.

## Notes

- These items are tracked as **potential behavioral regressions** identified in review, not confirmed production incidents.
- Reference report: `docs/REVIEW_REPORT_2026-05-04.md`.
