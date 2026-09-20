# Open-RMF Custom Modifications for NVIDIA Isaac Sim

This directory contains the source code for Open-RMF. While it is based on the standard `main` branch of Open-RMF, **several internal C++ logic files (.cpp / .hpp) have been tweaked** to accommodate the unique challenges of integrating with high-fidelity NVIDIA Isaac Sim physics.

## Why were tweaks needed?
In a standard Open-RMF deployment, the system assumes relatively instantaneous responses and idealized kinematics from the robots. However, because Isaac Sim is a heavy, constrained system running strict PhysX synchronization, small simulation delays and odometry drifts can cause the default Open-RMF scheduler to falsely trigger timeouts, deadlocks, or aggressive replanning. 

To tackle the delay from Isaac Sim under system constraints, we modified the core RMF logic to:
- Handle extended observation and synchronization delays.
- Prevent aggressive trajectory cancellations when costmaps lag behind physical movement.
- Improve negotiation handling when multiple AMRs suffer from simultaneous sim-time lag.

## Building this Workspace
You can build this workspace identically to standard Open-RMF:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

*Note: If you prefer, you can swap this folder out with a raw clone of the official Open-RMF repositories and build it in the exact same way. However, you may experience the simulation desync issues mentioned above.*
