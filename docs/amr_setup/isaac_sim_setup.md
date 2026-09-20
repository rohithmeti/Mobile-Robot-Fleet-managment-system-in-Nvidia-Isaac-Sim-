# NVIDIA Isaac Sim Environment & AMR Setup

## Environment Creation
The warehouse simulation was entirely built using NVIDIA Isaac Sim. We loaded standard warehouse assets (walls, rooms, racks, and props) to construct a realistic, physics-accurate factory floor. The final environment is saved as a `.usd` file (`Custom_Warehouse_lighting.usd`), which contains the physical mesh colliders, lighting, and spawn points for the AMRs.

## AMR Namespacing and Frame Isolation
To successfully run multiple AMRs (specifically five `iw_hub` industrial robots) on the same ROS 2 network, we had to strictly isolate their data streams. If all 5 AMRs published their odometry and lidar data to `/odom` and `/scan`, the ROS network would collapse under conflicting data.

**Isaac Sim OmniGraph Changes:**
Inside Isaac Sim, we modified the Action Graphs (OmniGraph) for each AMR to prepend a unique namespace to all published and subscribed ROS 2 topics. 
- **AMR 1:** Publishes to `/iw_hub_1/odom`, `/iw_hub_1/scan`, etc.
- **AMR 2:** Publishes to `/iw_hub_2/odom`, `/iw_hub_2/scan`, etc.

**TF Tree (Frame IDs):**
Similarly, we altered the ROS 2 TF publisher graphs inside Isaac Sim to broadcast isolated frame IDs for the robot links. Instead of standard `base_link`, the robots broadcast:
- `iw_hub_1/base_link`, `iw_hub_1/odom`, `iw_hub_1/front_lidar`
- `iw_hub_2/base_link`, `iw_hub_2/odom`, `iw_hub_2/front_lidar`

The only shared frame across all AMRs is the global `/map` frame, which acts as the absolute anchor for navigation.
