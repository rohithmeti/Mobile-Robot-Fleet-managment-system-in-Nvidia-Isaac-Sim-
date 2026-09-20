# Copyright 2021 Open Source Robotics Foundation, Inc.
# (Modified for IW.hub ROS 2 Native Integration - Fully Audited)

# ==============================================================================
# BLOCK 0: IMPORTS
# ==============================================================================
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.executors import SingleThreadedExecutor


# ==============================================================================
# BLOCK 1: MATH UTILITIES
# ==============================================================================
def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to yaw (radians)."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def quaternion_from_euler(ai, aj, ak):
    """Convert euler angles to quaternion [x, y, z, w]."""
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk
    return [cj*sc - sj*cs, cj*ss + sj*cc, cj*cs - sj*sc, cj*cc + sj*ss]


# ==============================================================================
# BLOCK 2: ROBOT API NODE (ROS2 interface to Nav2 and AMCL)
# ==============================================================================
class RobotAPI(Node):

    # --------------------------------------------------------------------------
    # BLOCK 2A: INIT — Sets up subscriptions, action clients, state dicts
    # --------------------------------------------------------------------------
    def __init__(self, config_yaml, use_sim_time=False):
        super().__init__('iw_hub_client_api')

        if use_sim_time:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'use_sim_time',
                    rclpy.parameter.Parameter.Type.BOOL,
                    True
                )
            ])

        self._lock = threading.Lock()

        # Load robot names from config
        fleet_dict = config_yaml.get('rmf_fleet', {})
        robots_dict = fleet_dict.get('robots', {})
        self.robot_names = list(robots_dict.keys())

        # State dictionaries — one entry per robot
        self.positions = {}          # [x, y, yaw]
        self.covariances = {}        # AMCL covariance[0] — 0.0 = not converged
        self.maps = {}               # map name string
        self.nav_clients = {}        # Nav2 action clients
        self.initial_pose_pubs = {}  # /initialpose publishers
        self.command_completed = {}  # True = Nav2 goal done or idle
        self.nav_succeeded_flag = {name: True for name in self.robot_names}
        self.goal_handles = {}       # Active Nav2 goal handles (for cancellation)
        
        # Initialize per-robot state
        for name in self.robot_names:
            self.positions[name] = None   # None = AMCL not received yet
            self.covariances[name] = 0.0  # 0.0 = not converged
            self.maps[name] = robots_dict.get(name, {}).get('initial_map', 'L1')
            self.command_completed[name] = True

        # QoS matching Nav2 AMCL (Transient Local — gets last message on connect)
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribe, create action clients, and publishers per robot
        for name in self.robot_names:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, n=name: self._pose_callback(msg, n),
                amcl_qos
            )
            self.nav_clients[name] = ActionClient(
                self, NavigateToPose, f'/{name}/navigate_to_pose'
            )
            self.initial_pose_pubs[name] = self.create_publisher(
                PoseWithCovarianceStamped,
                f'/{name}/initialpose',
                10
            )

        # Spin in background thread so callbacks fire without blocking main loop
        self.custom_executor = SingleThreadedExecutor()
        self.custom_executor.add_node(self)
        self.spin_thread = threading.Thread(
            target=self.custom_executor.spin, daemon=True
        )
        self.spin_thread.start()

    # --------------------------------------------------------------------------
    # BLOCK 2B: AMCL POSE CALLBACK — Updates position + covariance from AMCL
    # --------------------------------------------------------------------------
    def _pose_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        cov0 = msg.pose.covariance[0]  # x-variance; 0.0 = initial pose (not converged)
        with self._lock:
            self.positions[robot_name] = [x, y, yaw]
            self.covariances[robot_name] = cov0

    # --------------------------------------------------------------------------
    # BLOCK 2C: READ METHODS — position, covariance, map, battery, command state
    # --------------------------------------------------------------------------
    def position(self, robot_name: str):
        """Returns [x, y, yaw] or None if AMCL not received yet."""
        with self._lock:
            return self.positions.get(robot_name, None)

    def pose_covariance(self, robot_name: str):
        """Returns AMCL covariance[0]. 0.0 means AMCL not yet converged."""
        with self._lock:
            return self.covariances.get(robot_name, 0.0)

    def map(self, robot_name: str):
        with self._lock:
            return self.maps.get(robot_name, None)

    def battery_soc(self, robot_name: str):
        # Hardcoded 100% — replace with real topic if available
        return 1.0

    def is_command_completed(self, robot_name: str) -> bool:
        with self._lock:
            return self.command_completed.get(robot_name, False)

    def check_connection(self):
        return True

    def navigation_succeeded(self, robot_name: str) -> bool:
        with self._lock:
            return self.nav_succeeded_flag.get(robot_name, False)
    # --------------------------------------------------------------------------
    # BLOCK 2D: GET DATA — Bundles robot state into RobotUpdateData object
    # --------------------------------------------------------------------------
    def get_data(self, robot_name: str):
        """Returns RobotUpdateData if all fields available, else None."""
        map_str = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if map_str is None or position is None or battery_soc is None:
            return None
        return RobotUpdateData(robot_name, map_str, position, battery_soc)

    # --------------------------------------------------------------------------
    # BLOCK 2E: LOCALIZE — Publishes initial pose to AMCL
    # --------------------------------------------------------------------------
    def localize(self, robot_name: str, pose, map_name: str):
        if robot_name not in self.initial_pose_pubs:
            return False

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]

        quat = quaternion_from_euler(0.0, 0.0, pose[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07

        self.initial_pose_pubs[robot_name].publish(msg)

        with self._lock:
            self.maps[robot_name] = map_name
        return True

    # --------------------------------------------------------------------------
    # BLOCK 2F: NAVIGATE — Sends Nav2 NavigateToPose goal
    # --------------------------------------------------------------------------
    def navigate(self, robot_name: str, pose, map_name: str, speed_limit=0.0):
        if robot_name not in self.nav_clients:
            return False

        self.get_logger().info(f"[{robot_name}] Navigating to {pose}")
        with self._lock:
            self.command_completed[robot_name] = False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]

        quat = quaternion_from_euler(0.0, 0.0, pose[2])
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        client = self.nav_clients[robot_name]

        # Wait up to 5s for Nav2 action server to be ready
        server_ready = False
        for _ in range(10):
            if client.server_is_ready():
                server_ready = True
                break
            self.get_logger().warn(f"[{robot_name}] Waiting for Nav2 action server...")
            time.sleep(0.5)

        if not server_ready:
            self.get_logger().error(f"[{robot_name}] Nav2 action server not available!")
            with self._lock:
                self.command_completed[robot_name] = True
            return False

        send_goal_future = client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future, n=robot_name: self._goal_response_callback(future, n)
        )
        return True

    def _goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"[{robot_name}] Goal rejected by Nav2.")
            with self._lock:
                self.command_completed[robot_name] = True
            return
        with self._lock:
            self.goal_handles[robot_name] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, n=robot_name: self._goal_result_callback(future, n)
        )

    def _goal_result_callback(self, future, robot_name):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[{robot_name}] Reached destination.")
            with self._lock:
                self.nav_succeeded_flag[robot_name] = True
        else:
            self.get_logger().warn(
                f"[{robot_name}] Navigation failed or canceled. Status: {status}"
            )
            with self._lock:
                self.nav_succeeded_flag[robot_name] = False
        with self._lock:
            self.command_completed[robot_name] = True
    # --------------------------------------------------------------------------
    # BLOCK 2G: STOP — Cancels active Nav2 goal
    # --------------------------------------------------------------------------
    def stop(self, robot_name: str):
        with self._lock:
            handle = self.goal_handles.get(robot_name)
        if handle is not None:
            self.get_logger().info(f"[{robot_name}] Canceling current goal.")
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future, n=robot_name: self._cancel_done_callback(future, n)
            )
        return True

    def _cancel_done_callback(self, future, robot_name):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(f"[{robot_name}] Goal successfully canceled.")
        else:
            self.get_logger().warn(f"[{robot_name}] Goal failed to cancel.")

    # --------------------------------------------------------------------------
    # BLOCK 2H: ACTIVITY — Placeholder for tool/dispenser activities
    # --------------------------------------------------------------------------
    def start_activity(self, robot_name: str, activity: str, label: str):
        return True


# ==============================================================================
# BLOCK 3: ROBOT UPDATE DATA — Simple data container passed to RMF update loop
# ==============================================================================
class RobotUpdateData:
    def __init__(
        self,
        robot_name: str,
        map: str,
        position: list[float],
        battery_soc: float,
        requires_replan: bool | None = None
    ):
        self.robot_name = robot_name
        self.position = position      # [x, y, yaw]
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
