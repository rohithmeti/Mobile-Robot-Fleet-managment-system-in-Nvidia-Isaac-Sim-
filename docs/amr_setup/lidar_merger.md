# Dual-LiDAR Merging Setup

Each `iw_hub` AMR in our Isaac Sim environment is equipped with **two** RTX-accelerated 2D LiDARs (a front-left lidar and a rear-right lidar) to provide 360-degree coverage and eliminate blind spots. 

However, standard ROS 2 AMCL and Costmap2D nodes expect a single, unified `/scan` topic to calculate obstacles and localization.

## The `ira_laser_tools` Package
To solve this, we utilized the `ira_laser_tools` package. Our launch script, `merger_iw_hub_lidar.launch.py`, spins up 5 separate instances of the `laserscan_multi_merger` node—one for each robot namespace.

```python
Node(
    package='ira_laser_tools',
    executable='laserscan_multi_merger',
    name='laserscan_multi_merger',
    namespace='iw_hub_1',
    parameters=[merger_params_file],
    output='screen'
)
```

**How it works:**
1. The node subscribes to both `/iw_hub_1/front_lidar` and `/iw_hub_1/rear_lidar`.
2. It projects both 2D laser scans into the `iw_hub_1/base_link` coordinate frame.
3. It stitches the data together into a single, seamless 360-degree array.
4. It outputs the merged data to the `/iw_hub_1/scan_merged_1` topic, which is then safely ingested by the Nav2 costmap.
