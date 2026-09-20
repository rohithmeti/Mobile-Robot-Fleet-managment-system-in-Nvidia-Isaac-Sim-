# Mobile-Robot-Fleet-Management-System-in-NVIDIA-Isaac-Sim

## Overview
This repository contains the source code and configuration for simulating and validating multi-robot fleet coordination and dynamic task scheduling. The system integrates **NVIDIA Isaac Sim** for high-fidelity physics and environment rendering, the **ROS 2 Navigation Stack (Nav2)** for local autonomy, and **Open-RMF** for centralized fleet management and traffic deconfliction.

We demonstrate how a fleet of five Autonomous Mobile Robots (AMRs) can navigate a constrained warehouse environment, avoiding deadlocks in narrow aisles using a custom schedule-based traffic adapter.

## System Architecture

Instead of explaining *why* we did it, here is exactly *how* the simulation is constructed:

1. **Simulation Environment (Isaac Sim):** We use a realistic warehouse constructed from USD assets. We simulated 5 `iw_hub` industrial differential-drive AMRs. Each AMR is equipped with dual RTX-accelerated 2D LiDARs.
2. **ROS 2 Bridge & Navigation:** Data from Isaac Sim (sensor streams, odometry) is streamed directly to a ROS 2 network using the ROS Bridge. In the backend, each robot runs its own independent ROS 2 Navigation stack (Nav2) using AMCL for localization and DWB for local planning.
3. **Fleet Management (Open-RMF):** The entire fleet is orchestrated by Open-RMF, acting as the centralized traffic controller and dispatcher. Open-RMF assigns tasks based on bidding and handles space-time conflict resolution (deconfliction) so robots don't gridlock in narrow aisles. 

*Note: We built the entire Open-RMF stack from source rather than relying on standard `apt` installations. This was critical to avoid dependency errors and ensure stable performance across the distributed fleet adapter.*

## Setup and Launch Instructions

### 1. NVIDIA Isaac Sim Setup
1. **Download the Environment:** Download the Isaac Sim USD files and save it into a dedicated folder.
2. **Install Isaac Sim:** You must install **NVIDIA Isaac Sim version 4.5 or above** (mandatory for compatibility).
3. **Enable ROS Bridge:** Launch Isaac Sim and ensure the ROS 2 Bridge extension is enabled so that the simulation can communicate with the ROS network.
4. Load the `Custom_Warehouse_lighting.usd` environment.

### 2. ROS 2 Workspace Setup
This repository contains the ROS 2 packages required to run the AMRs.

```bash
# Clone this repository
git clone https://github.com/rohithmeti/Mobile-Robot-Fleet-managment-system-in-Nvidia-Isaac-Sim-.git
cd Mobile-Robot-Fleet-managment-system-in-Nvidia-Isaac-Sim-/ros2_team001

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the built workspace
source install/setup.bash
```
*Note: The `team_venv` folder contains the Python virtual environment with specific dependencies required for the simulation.*

### 3. Open-RMF Source Build Setup
To ensure compatibility and avoid common package manager errors, Open-RMF must be built from source. 
*(Note: We have made some specific changes to the RMF repositories to optimize them for this Isaac Sim environment, which will be explained in further documentation).*

To build Open-RMF from scratch, follow these steps:

1. **Create the workspace:**
   ```bash
   mkdir -p ~/Documents/open_rmf_source_build/src
   cd ~/Documents/open_rmf_source_build
   ```
2. **Import the source code:**
   ```bash
   wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
   vcs import src < rmf.repos
   ```
3. **Clean up unnecessary packages:**
   ```bash
   rm -rf src/rmf/rmf_simulation
   rm -rf src/demonstrations/rmf_demos
   ```
4. **Install dependencies:**
   ```bash
   sudo apt update && sudo apt install -y nlohmann-json3-dev libwebsocketpp-dev libasio-dev libssl-dev libboost-all-dev eigen3-progs libeigen3-dev ros-humble-ament-cmake-vendor-package
   source /opt/ros/humble/setup.bash
   rosdep update
   rosdep install --from-paths src --ignore-src --rosdistro humble --skip-keys="gz_fuel_tools_vendor gz_transport_vendor" -y
   ```
5. **Build Open-RMF (Option A - Standard Build):**
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
6. **Activate the environment:**
   ```bash
   source ~/Documents/open_rmf_source_build/install/setup.bash
   ```

Once Isaac Sim, the ROS 2 AMR workspace, and Open-RMF are all sourced and running, you can dispatch tasks to the fleet!
