# ROS 2 Architecture: Nodes, Topics, and Interfaces

> **Component-Based Swarm Design**
> 
> The MONA system is engineered using a modular, component-based architecture. This ensures high-performance intra-process communication (Zero-copy IPC) and strict logical separation between control, perception, and safety domains.

## 1. Swarm Namespace Isolation

> [!NOTE]
> Every robotic agent in the swarm operates within a dedicated, isolated ROS namespace (e.g., `/mona_001`, `/mona_002`). This prevents global topic collisions and allows the fleet orchestrator to address agents individually.

* **Global Topics:** `/tf`, `/tf_static`, `/map`, `/clock` (shared across the fleet).
* **Local Topics:** `/[ns]/odom`, `/[ns]/scan`, `/[ns]/cmd_vel` (isolated per robot).

---

## 2. Core Functional Domains

### 2.1. Safety, FDIR, and Fate Isolation (System Core)

To guarantee strict adherence to functional safety principles, the monolithic component container has been deprecated. The architecture now employs **Fate Isolation**, distributing node lifecycles across independent OS-level executors. This ensures that a fatal segmentation fault in expendable algorithmic layers cannot compromise the primary safety watchdog.

* **Nodes:**
  * `/mona_fdir_manager` (C++ Component, `mona_core`): The deterministic 10 Hz Lifecycle Manager (System Watchdog). It evaluates system health via non-blocking availability checks, executes the 3-Strikes hardware recovery protocol, and enforces Zero Velocity Overrides. Runs within the `isolated_fdir_executor`.
  * `/safety_node` (C++ Component, `mona_safety`): The hardware sentinel. It clamps velocities during degraded states, escalates anomalies detected via odometry, and physically opens hardware contactors during an E-STOP. Runs within the `isolated_safety_executor`.
* **Key Topics:**
  * `/system/health_state` (`mona_msgs/FdirState`): The current system health level broadcasted by FDIR (e.g., `NORMAL`, `DEGRADED`, `PROTECTIVE_STOP`, `EMERGENCY`).
  * `/system/safety_state` (`mona_msgs/SafetyState`): The current hardware safety state broadcasted by `safety_node`.
  * `/hardware/contactors` (`std_msgs/Bool`): The commanded state of the physical motor power relays.
  * `/hardware/motor_cmd` (`geometry_msgs/Twist`): The final velocity vector authorized by the safety gate.
* **Services:**
  * `/emergency_stop` (`std_srvs/Trigger`): Manually triggers a hard Level 2 E-STOP.
  * `/emergency_stop_reset` (`std_srvs/Trigger`): Clears the software E-STOP latch when reset conditions are satisfied.

### 2.2. Control and Multiplexing

Responsible for routing data streams and calculating motion kinematics before passing them to the safety layer.
* **Nodes:**
  * `/twist_mux_node` (C++ Component, `mona_control`): An intelligent velocity multiplexer. It arbitrates control between the autopilot and a human operator, applies Exponential Moving Average (EMA) smoothing, and preempts Nav2 goals during manual override.
* **Key Topics:**
  * `/cmd_vel_smoothed` (`geometry_msgs/Twist`): The final, smoothed velocity vector sent to the safety node.
  * `/mux_state` (`mona_msgs/TwistMuxState`): The active multiplexer state (`IDLE`, `MANUAL`, `AUTONOMOUS`, `BLOCKED_BY_SAFETY`).

### 2.3. Manual Control (Teleop)
* **Nodes:**
  * `/joy_node`: The gamepad driver (DualSense / Xbox).
  * `/teleop_twist_joy_node`: Translates raw gamepad button/axis inputs into velocity vectors.
* **Topics:**
  * `/joy` (`sensor_msgs/Joy`): Raw stick and button telemetry (100 Hz).
  * `/cmd_teleop` (`geometry_msgs/Twist`): Human teleoperation commands (Has the highest priority within the `twist_mux_node`).

### 2.4. Navigation and Planning (Nav2)
* **Nodes:**
  * `/controller_server`: The local planner (DWB). Tracks trajectories and dynamically avoids obstacles.
  * `/planner_server`: The global planner. Computes the optimal path using the static map.
  * `/bt_navigator`: The behavior tree coordinator orchestrating navigation states.
  * Additional nodes: `/smoother_server`, `/velocity_smoother`, `/waypoint_follower`.
* **Topics:**
  * `/goal_pose` (`geometry_msgs/PoseStamped`): The target destination coordinates.
  * `/cmd_nav` (`geometry_msgs/Twist`): Autopilot velocity commands (Automatically locked out for 5 seconds if gamepad input is detected).
  * `/plan` / `/local_plan` (`nav_msgs/Path`): Path visualizations for RViz2.

> [!NOTE]
> **LISA API Integration:** The global [LISA Fleet Planner](https://github.com/vladubase/lisa_api) dispatches target waypoints directly to the `/[namespace]/goal_pose` topic. MONA acknowledges the goal and streams execution status (Reached/Failed) back to LISA via standard Nav2 Action interfaces.

### 2.5. Perception and Fusion
* **Nodes:**
  * `/mona_lidar_merger` (C++ Component): Fuses point clouds from 4 independent LiDARs into a single spatial domain using Zero-Copy memory transfers.
  * `/pointcloud_to_laserscan`: Converts the fused 3D cloud back into a 2D scan for Nav2 consumption.
* **Topics:**
  * `/lidar_{front/back/left/right}/scan`: Raw simulation or hardware LiDAR streams.
  * `/perception/combined_cloud` (`sensor_msgs/PointCloud2`): The fused 3D point cloud.
  * `/scan` (`sensor_msgs/LaserScan`): The final 360-degree 2D scan consumed by Costmaps.
  * `/camera/image_raw` (`sensor_msgs/Image`): The raw video stream from the front-facing camera.
  * `/camera/camera_info` (`sensor_msgs/CameraInfo`): Camera calibration matrices.

### 2.6. Localization and Kinematics (EKF & SLAM)
* **Nodes:**
  * `/ekf_filter_node`: Fuses wheel odometry with IMU telemetry.
  * `/slam_toolbox`: Generates maps and localizes the robot within them.
  * `/robot_state_publisher`: Broadcasts the coordinate transformation tree (TF) based on the URDF.
* **Topics:**
  * `/odom`: Raw wheel odometry.
  * `/odom_filtered`: Smoothed odometry (IMU + Wheels).
  * `/map` (`nav_msgs/OccupancyGrid`): The warehouse map.
  * `/tf` and `/tf_static`: The robotic transformation tree.

---

## 3. Practical Guide: Debugging and Simulation

You can debug the robot's logic and simulate hardware behaviors directly from the terminal without relying on physical interfaces.

### 3.1. FDIR and Safety State Injection

The FDIR Manager continuously broadcasts the system health to the `/system/health_state` topic. While FDIR overrides external manual injection in production, during isolated debugging, you can observe the system's reaction to state escalations.

**Testing Degradation (Auxiliary Node Failure):**
If a non-critical sensor fails, FDIR transmits `STATE_DEGRADED` (ID: 2). The robot should immediately cap its maximum velocity.
```bash
ros2 topic pub --once /mona_001/system/health_state mona_msgs/msg/FdirState "{current_state: 2, diagnostic_message: 'debug degraded injection'}"
```

**Testing E-STOP & Zero Velocity Override (Fatal Failure):**
Simulate a critical node crash or a physical red button press (`STATE_EMERGENCY`, ID: 5).
```bash
ros2 topic pub --once /mona_001/system/health_state mona_msgs/msg/FdirState "{current_state: 5, diagnostic_message: 'debug emergency injection'}"
```
_Expected Result:_ FDIR instantly seizes the control pipeline and publishes a Zero Velocity Override. The `safety_node` halts all operations and the terminal logs `HARDWARE: Contactors OPENED. Motors DEAD.`

**Restoring Operation:**
```bash
ros2 topic pub --once /mona_001/system/health_state mona_msgs/msg/FdirState "{current_state: 1, diagnostic_message: 'debug normal injection'}"
```

### 3.2. Lifecycle Management

Core nodes (`safety_node`, `mona_lidar_merger`, and the entire Nav2 stack) utilize the ROS 2 Managed Node (Lifecycle) architecture.

**Check Current Node State:**
```bash
ros2 lifecycle get /mona_001/safety_node
```
_(Expected responses: `active`, `inactive`, or `unconfigured`)_

**Manually Suspend Data Processing (Pause):**
If you need the robot to ignore control commands without terminating the underlying process, transition the safety node to `inactive`:
```bash
ros2 lifecycle set /mona_001/safety_node deactivate
```
_Expected Result:_ The node stops publishing to `hardware/motor_cmd`, and hardware contactors are softly disengaged.

**Resume Operation:**
```bash
ros2 lifecycle set /mona_001/safety_node activate
```

**Monitor all state transitions via the event topic:**
```bash
ros2 topic echo /mona_001/safety_node/transition_event
```

### 3.3. Motion Simulation (Without Gamepad)

You can move the robot by directly emulating gamepad or autopilot telemetry.

**Teleoperator Signal (Triggers 5-second Nav2 lockout):**
```bash
ros2 topic pub --rate 20 /mona_001/cmd_teleop geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.0}}"
```

**Autopilot Signal:**
```bash
ros2 topic pub --rate 10 /mona_001/cmd_nav geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.0}}"
```

> [!NOTE]
> If you publish to `cmd_teleop` and immediately follow up with `cmd_nav`, the latter will be completely ignored due to the `manual_timeout_` logic (gamepad input enforces a 5-second block on all autonomous commands).

### 3.4. Hardware Verification

To verify that the safety logic is successfully triggering physical motor contactors, monitor the hardware command topic:
```bash
ros2 topic echo /mona_001/hardware/contactors
```
- `data: true` -> Relay is closed. Power (24V/48V) is supplied to the motor bridge drivers.
- `data: false` -> Relay is open. Motors are de-energized (hardware brakes will engage).
