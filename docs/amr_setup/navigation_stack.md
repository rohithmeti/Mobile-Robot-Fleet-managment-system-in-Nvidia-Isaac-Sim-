# ROS 2 Navigation Stack (Nav2) Multi-Robot Setup

To control 5 independent AMRs, we require 5 independent instances of the Nav2 stack running simultaneously. This is achieved using the `iw_hub_navigation` package.

## Launch File Architecture
In `multi_iw_hub_navigation.launch.py`, we define a list of the robots:
```python
robots = [
    {'name': 'iw_hub_1', 'id': '1', 'params': 'iw_hub_1_nav2_params.yaml'},
    {'name': 'iw_hub_2', 'id': '2', 'params': 'iw_hub_2_nav2_params.yaml'},
    {'name': 'iw_hub_3', 'id': '3', 'params': 'iw_hub_3_nav2_params.yaml'},
    {'name': 'iw_hub_4', 'id': '4', 'params': 'iw_hub_4_nav2_params.yaml'},
    {'name': 'iw_hub_5', 'id': '5', 'params': 'iw_hub_5_nav2_params.yaml'}
]
```
The launch file loops through this array and uses `PushRosNamespace` to spawn a completely isolated Nav2 Bringup instance for each robot under its respective namespace (`/iw_hub_1`, `/iw_hub_2`, etc.). 

## Parameter File Customization
A standard Nav2 parameter file expects global TF frames like `base_link`. We duplicated the parameter file 5 times (`iw_hub_1_nav2_params.yaml`, etc.) and manually mapped the TF frames to match the Isaac Sim outputs:

```yaml
# Example from iw_hub_1_nav2_params.yaml
amcl:
  ros__parameters:
    base_frame_id: "iw_hub_1/base_link"
    global_frame_id: "map"
    odom_frame_id: "iw_hub_1/odom"
```

This strict mapping ensures that `iw_hub_1`'s AMCL node only calculates localization using its own LiDAR and odometry data, completely ignoring the other 4 robots.
