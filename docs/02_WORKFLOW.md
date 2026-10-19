# Daily Workflow

> **Build, Test, Repeat**
> 
> System stability is guaranteed through regular automated testing and strict build hygiene.

## 1. Project Build Sequence
Within our distributed Swarm architecture, the primary workspace compilation (`colcon build`) is executed within the Docker image. This guarantees a deterministic environment across all swarm agents.

### Incremental Builds (Development)
To apply changes to C++ modules without rebuilding the entire Docker image, utilize the `dev` service:
```bash
docker compose exec dev bash
colcon build --symlink-install
```
The `--symlink-install` flag enables dynamic reloading of Python scripts and configuration files (.yaml, .xacro, .launch.py) on the host machine without requiring a recompilation.

### Clean Builds (Infrastructure Updates)
A full rebuild is mandatory when modifying system dependencies in `package.xml` or updating the `Dockerfile`:
```bash
make rebuild
# OR
docker compose build --no-cache
```

---

## 2. Environment Sourcing
After every fresh compilation inside the `dev` container, you must source the environment overlays to expose newly built packages to the ROS 2 CLI:
```bash
source install/setup.bash
```

---

## 3. Continuous Integration (Local CI Quality Gate)

Before submitting a Pull Request, all code must pass the stringent local Continuous Integration (CI) pipeline.
This guarantees conformity to our memory safety standards and FSM lifecycle contracts.

Execute the unified quality gate directly from the host machine utilizing the root Makefile:
```bash
# Formats code (Uncrustify, Black) and verifies it with the CI linters
# (uncrustify, cpplint, cppcheck, flake8, lint_cmake, xmllint, copyright)
make format_code

# Builds and runs all GTest suites plus the linters
make ci
```

Alternatively, invoke the validation script directly within the active development container:
```bash
docker compose exec -it dev bash

./scripts/format_code.bash

./scripts/run_ci_checks.bash
```

> [!WARNING]
> Bypassing the local CI gate will result in an automated rejection of your Pull Request by the GitHub Actions pipeline. Ensure tests successfully passing locally.

---

## 4. Dependency Management (rosdep)
`rosdep` is a package manager abstraction tool. It parses the `package.xml` files across all your packages and installs the corresponding OS-level dependencies (e.g., via `apt`).
**When to execute:**
1. During the initial project setup.
2. If a colleague introduces a new library in `package.xml` and your build fails with a `Could not find a package configuration file provided by...` error.

**Command:**
```bash
# --from-paths src: Scan the src/ directory
# --ignore-src: Do not attempt to install packages we are actively developing
# -r: Continue installing despite minor errors
# -y: Automatically answer "yes" to apt-get prompts

rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

---

## 5. Troubleshooting Common Issues

#### The Map Freezes on the First Frame (Lock on Chassis)
- **Root Cause:** The SLAM algorithm is not receiving displacement updates (Odometry failure or a disconnected `odom -> base_footprint` TF tree).
- **Resolution:** Verify the odometry stream using `ros2 topic echo /mona_001/odom` (ensure you respect the agent's namespace). If data is absent, check the `OdometryPublisher` plugin parameters within the URDF. Additionally, verify that the global `/clock` topic is properly bridged via `ros_gz_bridge` so Gazebo and ROS 2 share the same simulation time.

#### WARNING: colcon.colcon_ros.task.ament_python.build
- **Root Cause:** You are attempting to utilize `--symlink-install` on Python packages that rely on standard `setup.py` configurations, which can occasionally trigger deprecation warnings in newer `setuptools` versions.
- **Resolution:** This warning is benign and does not affect runtime execution. It can be safely ignored during local development.
