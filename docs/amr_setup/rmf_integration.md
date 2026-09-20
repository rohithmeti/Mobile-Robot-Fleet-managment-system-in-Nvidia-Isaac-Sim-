# Open-RMF Integration Overview

For detailed technical explanations regarding how we integrated Open-RMF into this multi-AMR setup, please refer to the specific README files located at the roots of their respective packages:

1. **RMF Core Tweaks & Latency Handling:** 
   [Read the RMF Setup Guide](../../ros2_team001/src/rmf/README.md) - *Details the C++ adjustments made to handle Isaac Sim FPS drops and negotiation depth.*
   
2. **Fleet Adapter Configuration:**
   See the `iw_hub_fleet_config.yaml` located in `ros2_team001/src/iw_fleet_adapter/` for details on how the `RobotClientAPI` translates RMF goals into specific `NavigateToPose` commands targeted at the `/iw_hub_X` Nav2 namespaces.
