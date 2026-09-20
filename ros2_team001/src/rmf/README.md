# Custom Open-RMF Modifications for NVIDIA Isaac Sim

This directory contains our customized source code for Open-RMF. While it is based on the standard `main` branch of Open-RMF, **several internal C++ logic files (.cpp / .hpp) across the core packages have been heavily tweaked.**

## Why were tweaks needed? (Simulation vs. Reality)

The standard raw Open-RMF release (such as the one found in our `ros2_fleet` repository) is explicitly tuned for **actual, physical AMRs** operating in the real world. Real-world systems generally have predictable sensor timing and do not suffer from "simulation clock lag."

However, because NVIDIA Isaac Sim is a heavy, GPU-constrained system running strict PhysX synchronization, the simulation often suffers from slight frame drops (FPS issues) and odometry latency. These micro-delays cause the default Open-RMF scheduler to falsely trigger timeouts, deadlocks, or aggressive replanning, as it assumes the robots are failing to move or have disconnected.

To tackle this simulation-induced latency and prevent fleet adapter disconnections, we modified the core RMF logic within `ros2_team001`. 

### Deep Dive: Specific Code Changes Made

1. **`rmf_traffic` (Schedule Negotiation Depth):**
   * **`CentralizedNegotiation.cpp`**: Under heavy simulation load, multiple AMRs arriving at a shared intersection simultaneously will trigger an RMF space-time negotiation. In the raw code, if the negotiation `table->version() > 2`, it aborts, leading to deadlock. **We increased this depth threshold to `> 6`**, allowing the traffic planner to survive extended negotiation rounds caused by Isaac Sim's processing latency without crashing the schedule.

2. **`rmf_fleet_adapter` (Traffic Light & Delay Propagation):**
   * **`EasyTrafficLight.cpp`**: We disabled the default cumulative delay propagation (`existing_cumulative_delay + delay_delta`). Instead, the delay is strictly reset to `hooks.node->rmf_now() - expected_time`. This prevents the fleet adapter from mathematically snowballing micro-stutters into massive artificial delays that would otherwise force the global schedule to eject the robot.

3. **Fleet Adapter Configuration & RobotClient API:**
   * **`iw_hub_fleet_config.yaml`**: The standard RMF fleet state publication rate is typically 1Hz - 2Hz. We drastically bumped both `publish_fleet_state` and `robot_state_update_frequency` to **20.0 Hz**. This aggressive polling rate guarantees that the `RobotClientAPI` is constantly pumping odometry updates to the RMF core. If Isaac Sim drops a few frames, the high update frequency ensures the fleet adapter does not falsely assume the robot has disconnected and drop its API connection.

## Building this Workspace
You can build this workspace identically to standard Open-RMF. If you clone this repository, you simply run:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

*Note: If you are deploying to **physical AMRs**, you can safely delete this customized `rmf` folder, clone the raw official Open-RMF repositories in its place, and build it identically. The WebSockets and Python bindings will compile perfectly either way.*
