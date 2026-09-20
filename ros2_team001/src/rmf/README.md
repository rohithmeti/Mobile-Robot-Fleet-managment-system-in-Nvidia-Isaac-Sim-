# Custom Open-RMF Modifications for NVIDIA Isaac Sim

This directory contains our customized source code for Open-RMF. While it is based on the standard `main` branch of Open-RMF, **several internal C++ logic files (.cpp / .hpp) across the core packages have been heavily tweaked.**

## Why were tweaks needed? (Simulation vs. Reality)

The standard raw Open-RMF release (such as the one found in our `ros2_fleet` repository) is explicitly tuned for **actual, physical AMRs** operating in the real world. Real-world systems generally have predictable sensor timing and do not suffer from "simulation clock lag."

However, because NVIDIA Isaac Sim is a heavy, GPU-constrained system running strict PhysX synchronization, the simulation often suffers from slight frame drops (FPS issues) and odometry latency. These micro-delays cause the default Open-RMF scheduler to falsely trigger timeouts, deadlocks, or aggressive replanning, as it assumes the robots are failing to move.

To tackle this simulation-induced latency, we modified the core RMF logic within `ros2_team001/src/rmf`. 

### Key C++ / HPP Changes Made:

1. **`rmf_fleet_adapter` (The Core Logic):**
   * **`agv/RobotUpdateHandle.cpp` & `agv/RobotContext.cpp`**: We increased the internal tolerance thresholds for positional tracking. If the Isaac Sim odometry temporarily desynchronizes from the Nav2 costmap, the adapter will wait longer before declaring the robot "lost" or issuing an abort command.
   * **`agv/Node.cpp` & `EasyFullControl.cpp`**: We adjusted the trajectory timeout variables. The fleet adapter now accommodates sudden FPS drops in Isaac Sim without immediately throwing a `Status 6 (Abort)` error and abandoning the goal.
2. **`rmf_traffic` (Schedule Negotiation):**
   * **`agv/CentralizedNegotiation.cpp`**: When multiple simulated AMRs experience lag simultaneously at a congested intersection, the default negotiator fails fast. We extended the depth and retry counts of the negotiation logic to allow robots to properly resolve space-time overlaps despite odometry jitter.
3. **`rmf_visualization` (RViz Map Syncing):**
   * **`FloorplanVisualizer.cpp`**: We adjusted how the floorplans and nav-graphs render to ensure they don't crash RViz2 when the `use_sim_time:=true` clock fluctuates wildly under heavy GPU load.

## Building this Workspace
You can build this workspace identically to standard Open-RMF. If you clone this repository, you simply run:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

*Note: If you are deploying to **physical AMRs**, you can safely delete this customized `rmf` folder, clone the raw official Open-RMF repositories in its place, and build it identically. The WebSockets will compile either way.*
