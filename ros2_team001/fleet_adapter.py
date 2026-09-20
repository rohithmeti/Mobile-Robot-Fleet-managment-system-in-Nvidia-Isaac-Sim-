import sys
import argparse
import yaml
import time
import threading
import nudged
import datetime

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter
from rmf_adapter import Adapter
from rmf_adapter import vehicletraits as traits
from rmf_adapter import geometry, battery, plan

from .RobotClientAPI import RobotAPI

# ==============================================================================
# BLOCK 1: COMMAND HANDLE
# ==============================================================================
class IWHubCommandHandle(rmf_adapter.RobotCommandHandle):
    def __init__(self, name, api, node, transform):
        rmf_adapter.RobotCommandHandle.__init__(self)
        self.name = name
        self.api = api
        self.node = node
        self.transform = transform
        self.update_handle = None
        self._current_path_id = 0
        self._lock = threading.Lock()

    def stop(self):
        with self._lock:
            self._current_path_id += 1
        self.node.get_logger().info(f"[{self.name}] Open-RMF issued STOP. Preempting current path.")
        self.api.stop(self.name)

    def follow_new_path(self, waypoints, arrival_cb, path_finished_cb):
        with self._lock:
            self._current_path_id += 1
            path_id = self._current_path_id

        self.node.get_logger().info(f"[{self.name}] New path received with {len(waypoints)} waypoints.")

        def monitor():
            # FIX A: Wait for Nav2 to finish processing any previous goal cancellation.
            # stop() sends cancel_goal_async (fire-and-forget). Without this sleep,
            # navigate() fires before Nav2 processes the cancel -> immediate Status 6.
            time.sleep(0.4)

            prev_pos = None
            for i, wp in enumerate(waypoints):
                with self._lock:
                    if self._current_path_id != path_id:
                        return

                pos = [wp.position[0], wp.position[1], wp.position[2]]
                delay = datetime.timedelta(seconds=0)

                # Hold-in-place: LAST waypoint only, same xy as previous
                is_last_waypoint = (i == len(waypoints) - 1)
                if is_last_waypoint and prev_pos is not None:
                    if abs(pos[0] - prev_pos[0]) < 0.05 and abs(pos[1] - prev_pos[1]) < 0.05:
                        self.node.get_logger().info(f"[{self.name}] Hold-in-place at final waypoint {i}: {pos}")
                        while True:
                            with self._lock:
                                if self._current_path_id != path_id:
                                    return
                            time.sleep(0.5)

                # FIX C: Skip Nav2 for yaw-only waypoints.
                # RMF sends orientation-adjustment waypoints at the robot's current xy.
                # DWB handles these badly when TF is slightly stale -> Status 6.
                # If robot is already within 0.3m of the target xy, skip Nav2
                # and immediately fire arrival_cb.
                robot_data = self.api.get_data(self.name)
                if robot_data is not None:
                    rx, ry = robot_data.position[0], robot_data.position[1]
                    if abs(pos[0] - rx) < 0.3 and abs(pos[1] - ry) < 0.3:
                        self.node.get_logger().info(
                            f"[{self.name}] wp {i} is yaw-only (robot within 0.3m) — skipping Nav2"
                        )
                        with self._lock:
                            if self._current_path_id != path_id:
                                return
                        arrival_cb(i, delay)
                        prev_pos = pos
                        continue

                # FIX B: Retry navigation up to 3 times per waypoint.
                MAX_RETRIES = 3
                nav_succeeded = False

                for attempt in range(MAX_RETRIES):
                    with self._lock:
                        if self._current_path_id != path_id:
                            return

                    self.node.get_logger().info(f"[{self.name}] Navigating to waypoint {i}: {pos}")
                    self.api.navigate(self.name, pos, "L1")

                    while not self.api.is_command_completed(self.name):
                        with self._lock:
                            if self._current_path_id != path_id:
                                return
                        time.sleep(0.2)

                    with self._lock:
                        if self._current_path_id != path_id:
                            return

                    # Check if robot actually reached the waypoint (within 0.5m)
                    robot_data = self.api.get_data(self.name)
                    if robot_data is not None:
                        rx, ry = robot_data.position[0], robot_data.position[1]
                        dist_sq = (pos[0] - rx) ** 2 + (pos[1] - ry) ** 2
                        if dist_sq < 0.5 * 0.5:
                            nav_succeeded = True
                            break
                        else:
                            self.node.get_logger().warn(
                                f"[{self.name}] Nav ABORTED at wp {i}, retry {attempt + 1}/{MAX_RETRIES}"
                            )
                            if attempt < MAX_RETRIES - 1:
                                time.sleep(0.3)
                    else:
                        # No pose data — treat as succeeded to avoid blocking
                        nav_succeeded = True
                        break

                if not nav_succeeded:
                    # FIX B (cont): All retries exhausted — ABANDON PATH.
                    # Skipping ahead desynchs RMF's belief of robot position,
                    # causing RMF to replan with 'backtracking' corrective paths.
                    # By returning without path_finished_cb, RMF times out
                    # and replans from the robot's actual position.
                    self.node.get_logger().warn(
                        f"[{self.name}] Abandoning path — failed to reach wp {i} after {MAX_RETRIES} retries"
                    )
                    return

                arrival_cb(i, delay)
                prev_pos = pos

            # All waypoints completed
            with self._lock:
                if self._current_path_id == path_id:
                    path_finished_cb()

        threading.Thread(target=monitor, daemon=True).start()

    def dock(self, dock_name, docking_finished_cb):
        self.node.get_logger().info(f"[{self.name}] Docking at {dock_name}")
        docking_finished_cb()


# ==============================================================================
# BLOCK 2: TRANSFORMS
# ==============================================================================
def compute_transforms(coords, node=None):
    rmf_coords = coords['rmf']
    robot_coords = coords['robot']
    tf = nudged.estimate(rmf_coords, robot_coords)
    if node:
        mse = nudged.estimate_error(tf, rmf_coords, robot_coords)
        node.get_logger().info(f"MSE: {mse}")
    return tf


# ==============================================================================
# BLOCK 3: MAIN ENTRY & CONFIGURATION
# ==============================================================================
def main(argv=sys.argv):
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()

    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config_file", type=str, required=True)
    parser.add_argument("-n", "--nav_graph", type=str, required=True)
    parser.add_argument("-sim", "--use_sim_time", action="store_true")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])

    with open(args.config_file, "r") as f:
        config_yaml = yaml.safe_load(f)

    fleet_name = config_yaml['rmf_fleet']['name']
    node = rclpy.node.Node(f'{fleet_name}_command_handle')
    adapter = Adapter.make(f'{fleet_name}_fleet_adapter')

    if args.use_sim_time:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        adapter.node.use_sim_time()


# ==============================================================================
# BLOCK 4: FLEET INITIALIZATION & TRAITS
# ==============================================================================
    profile = traits.Profile(
        geometry.make_final_convex_circle(config_yaml['rmf_fleet']['profile']['footprint']),
        geometry.make_final_convex_circle(config_yaml['rmf_fleet']['profile']['vicinity'])
    )

    limits = config_yaml['rmf_fleet']['limits']
    linear_limits = traits.Limits(limits['linear'][0], limits['linear'][1])
    angular_limits = traits.Limits(limits['angular'][0], limits['angular'][1])

    v_traits = traits.VehicleTraits(
        linear=linear_limits,
        angular=angular_limits,
        profile=profile
    )
    v_traits.differential.reversible = config_yaml['rmf_fleet'].get('reversible', False)

    graph = rmf_adapter.graph.parse_graph(args.nav_graph, v_traits)
    fleet_handle = adapter.add_fleet(fleet_name, v_traits, graph)

    publish_freq = config_yaml['rmf_fleet'].get('robot_state_update_frequency', 10.0)
    fleet_handle.fleet_state_publish_period(
        datetime.timedelta(seconds=1.0 / publish_freq)
    )

    b_sys = battery.BatterySystem.make(24.0, 40.0, 0.2)
    mech_sys = battery.MechanicalSystem.make(70.0, 40.0, 0.22)
    ambient_sys = battery.PowerSystem.make(20.0)
    tool_sys = battery.PowerSystem.make(10.0)

    motion_sink = battery.SimpleMotionPowerSink(b_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(b_sys, ambient_sys)
    tool_sink = battery.SimpleDevicePowerSink(b_sys, tool_sys)

    finishing_request = config_yaml['rmf_fleet'].get('finishing_request', 'charge')
    drain_battery = config_yaml['rmf_fleet'].get('account_for_battery_drain', True)

    ok = fleet_handle.set_task_planner_params(
        b_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        config_yaml['rmf_fleet']['recharge_threshold'],
        config_yaml['rmf_fleet']['recharge_soc'],
        drain_battery,
        finishing_request
    )
    if not ok:
        node.get_logger().error("set_task_planner_params FAILED")

    fleet_handle.accept_task_requests(lambda request: True)

    def _consider_patrol(json_desc):
        confirmation = rmf_adapter.fleet_update_handle.Confirmation()
        confirmation.accept()
        return confirmation
    fleet_handle.consider_patrol_requests(_consider_patrol)

    api = RobotAPI(config_yaml=config_yaml, use_sim_time=args.use_sim_time)


# ==============================================================================
# BLOCK 5: ADD ROBOTS
# ==============================================================================
    robots = {}
    tf_data = config_yaml['reference_coordinates']['L1']
    tf = compute_transforms(tf_data, node)

    for robot_name in config_yaml['rmf_fleet']['robots']:
        cmd_handle = IWHubCommandHandle(robot_name, api, node, tf)

        data = api.get_data(robot_name)
        while data is None:
            node.get_logger().info(f"Waiting for initial pose of {robot_name}...")
            time.sleep(1.0)
            data = api.get_data(robot_name)

        # Discard [0,0] — Isaac Sim boot default before AMCL converges
        while abs(data.position[0]) < 0.01 and abs(data.position[1]) < 0.01:
            node.get_logger().warn(f"[{robot_name}] Pose is [0,0] — waiting for AMCL to converge...")
            time.sleep(1.0)
            data = api.get_data(robot_name)

        min_dist = float('inf')
        closest_wp_idx = 0
        for i in range(graph.num_waypoints):
            wp = graph.get_waypoint(i)
            if wp.map_name != data.map:
                continue
            dx = wp.location[0] - data.position[0]
            dy = wp.location[1] - data.position[1]
            dist = dx * dx + dy * dy
            if dist < min_dist:
                min_dist = dist
                closest_wp_idx = i

        node.get_logger().info(f"[{robot_name}] Snapped to graph waypoint {closest_wp_idx} on {data.map}")

        now = adapter.now()
        yaw_float = float(data.position[2])
        starts = [plan.Start(now, closest_wp_idx, yaw_float)]

        def handle_cb(handle, c=cmd_handle, robot_n=robot_name):
            c.update_handle = handle
            charger_name = config_yaml['rmf_fleet']['robots'][robot_n].get('charger', '')
            charger_wp_idx = 0
            for i in range(graph.num_waypoints):
                if graph.get_waypoint(i).waypoint_name == charger_name:
                    charger_wp_idx = i
                    break
            handle.set_charger_waypoint(charger_wp_idx)
            node.get_logger().info(f"[{robot_n}] Charger set to '{charger_name}' = waypoint {charger_wp_idx}")

        fleet_handle.add_robot(cmd_handle, robot_name, profile, starts, handle_cb)
        robots[robot_name] = cmd_handle

    adapter.start()


# ==============================================================================
# BLOCK 6: UPDATE LOOP & SPIN
# ==============================================================================
    WAYPOINT_SNAP_THRESHOLD_SQ = 0.5 * 0.5

    def update_loop():
        while rclpy.ok():
            for name, handle in robots.items():
                try:
                    data = api.get_data(name)
                    if data and handle.update_handle is not None:
                        now = adapter.now()
                        x = float(data.position[0])
                        y = float(data.position[1])
                        yaw = float(data.position[2])

                        min_dist_sq = float('inf')
                        closest_wp_idx = 0
                        for i in range(graph.num_waypoints):
                            wp = graph.get_waypoint(i)
                            if wp.map_name != data.map:
                                continue
                            dx = wp.location[0] - x
                            dy = wp.location[1] - y
                            dist_sq = dx * dx + dy * dy
                            if dist_sq < min_dist_sq:
                                min_dist_sq = dist_sq
                                closest_wp_idx = i

                        if min_dist_sq > WAYPOINT_SNAP_THRESHOLD_SQ:
                            start = plan.Start(now, closest_wp_idx, yaw, location=[x, y])
                        else:
                            start = plan.Start(now, closest_wp_idx, yaw)

                        handle.update_handle.update_position([start])
                except Exception as e:
                    node.get_logger().error(f"Update loop error for {name}: {e}")

            time.sleep(0.1)

    threading.Thread(target=update_loop, daemon=True).start()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
