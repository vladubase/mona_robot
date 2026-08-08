# Foxglove Studio Dispatch Guide

> **Centralized Fleet Telemetry**
> 
> Foxglove Studio serves as the primary interface for monitoring, debugging, and dispatching the MONA robot fleet. We utilize a custom dashboard layout that integrates navigation visualization, FDIR (Fault Tolerance) health monitoring, and hardware interlock controls.

## 1. Establishing a Connection

Whenever the simulation infrastructure is launched (`./scripts/start_world.bash`), the `foxglove_bridge` node is automatically initialized. It streams ROS 2 telemetry over a WebSocket connection.
1. Open the **Foxglove Studio** desktop application (or the web interface).
2. Navigate to the **Layouts** menu (the two-squares icon on the left sidebar).
3. Click **Import from file...**
4. Select the layout file from the repository: `src/mona_core/configs/foxglove_dashboard.json`.
5. Click **Open Connection**.
6. Select the **Foxglove WebSocket** protocol.
7. Enter the WebSocket URL: `ws://127.0.0.1:8765` (for local simulation) or the specific IP address of your physical robot/host.
8. Click **Open**.

---

## 2. Dashboard Interface Overview

![Foxglove preview](/docs/images/foxglove_preview.png)

### The layout is partitioned into distinct functional zones:

**Visualization (Central Panel):**
* **3D Panel:** Renders the active coordinate transform tree (TF), LiDAR point clouds (`/[namespace]/scan`), the global warehouse map (`/map`), and navigation trajectories (`/[namespace]/plan`, `/[namespace]/local_plan`). It is strictly anchored to the `map` frame to visualize absolute global positioning.

**Diagnostics and Telemetry (Right Panel):**
* **Raw Messages:** A live data stream of critical system topics.
  * `/[namespace]/system/health_state`: The FDIR health status.
  * `/[namespace]/system/safety_state`: The SafetyNode hardware safety state.
  * `/[namespace]/hardware/contactors`: The physical state of the motor relays.
* **Diagnostics:** A structured hierarchical view of the ROS 2 Diagnostics pipeline. Displays hardware errors, operational frequency drops, and safety system statuses.

**Logs and Kinematic Plots (Bottom Panel):**
* **Log:** The standard output stream (ROSout). Essential for tracking `RCLCPP` warnings, errors, and state transitions.
* **Plot (Linear X-Y / Angular Z):** Real-time comparative graphs plotting raw input commands (`/[namespace]/cmd_teleop`, `/[namespace]/cmd_nav`) against the final, filtered physical velocity (`/[namespace]/hardware/motor_cmd`). This is the primary tool for tuning PID controllers and the EMA filter alpha.

---

## 3. Safety-Critical Architecture Features

When analyzing data in Foxglove, keep the following architectural paradigms in mind:

1. **Topic Collision Protection (Race Conditions):** The `twist_mux_node` is strictly prohibited from publishing directly to the global `/robot_status` topic. It broadcasts its internal state to `/[namespace]/mux_state`, while the `safety_node` aggregates health and motion data before authorizing hardware commands.
2. **Zero-Spam Deadzone Filter:** When a human operator releases the gamepad analog sticks, the joystick driver floods the network with zero-velocity commands (`Twist: 0.0`). The multiplexer intelligently recognizes this deadzone state. It drops the zero-spam and **does not extend** the manual lockout timer, allowing the Nav2 autopilot to seamlessly regain control once the 5-second timeout expires.
3. **E-STOP Absolute Priority:** A triggered `EMERGENCY` state possesses the highest system priority. When active, all commands from the gamepad or Nav2 are discarded at the C++ execution level, and the target velocity is rigidly clamped to zero, followed by a hardware contactor cutoff.

---

## 4. Saving Layout Modifications

If you introduce a new panel, modify plot configurations, or adjust data formats and wish to persist these changes for the rest of the development team:
1. Open the **Layouts** menu in Foxglove Studio.
2. Click the **Download** button to save the active layout as a JSON file.
3. Overwrite the existing repository artifact at `src/mona_core/configs/foxglove_dashboard.json` with your newly downloaded file.
4. Commit and push the updated layout via Git.
