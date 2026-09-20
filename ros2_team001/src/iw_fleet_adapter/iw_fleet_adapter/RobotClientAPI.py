# Copyright 2021 Open Source Robotics Foundation, Inc.
# (Modified for IW.hub ROS 2 Native Integration - EasyFullControl Verified)

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
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def quaternion_from_euler(ai, aj, ak):
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
    def __init__(self, config_yaml, use_sim_time=False):
        super().__init__('iw_hub_client_api')
        if use_sim_time:
            self.set_parameters([
                rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
            ])
        self._lock = threading.Lock()
        
        fleet_dict = config_yaml.get('rmf_fleet', {})
        robots_dict = fleet_dict.get('robots', {})
        self.robot_names = list(robots_dict.keys())
        
        # New State Tracking Variables
        self._cmd_id = {}       # robot -> int, bumped on every navigate/stop
        self._goal_handle = {}
        self._nav_done = {}
        self._dest = {}

        self.positions = {}          
        self.covariances = {}        
        self.maps = {}               
        self.nav_clients = {}        
        self.initial_pose_pubs = {}  

        for name in self.robot_names:
            self.positions[name] = None   
            self.covariances[name] = 0.0  
            self.maps[name] = robots_dict.get(name, {}).get('initial_map', 'L1')
            self._nav_done[name] = False

        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        for name in self.robot_names:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, n=name: self._pose_callback(msg, n),
                amcl_qos
            )
            self.nav_clients[name] = ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose')
            self.initial_pose_pubs[name] = self.create_publisher(PoseWithCovarianceStamped, f'/{name}/initialpose', 10)

        self.custom_executor = SingleThreadedExecutor()
        self.custom_executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.custom_executor.spin, daemon=True)
        self.spin_thread.start()

    def _pose_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        cov0 = msg.pose.covariance[0]  
        with self._lock:
            self.positions[robot_name] = [x, y, yaw]
            self.covariances[robot_name] = cov0

    def position(self, robot_name: str):
        with self._lock:
            return self.positions.get(robot_name, None)

    def pose_covariance(self, robot_name: str):
        with self._lock:
            return self.covariances.get(robot_name, 0.0)

    def map(self, robot_name: str):
        with self._lock:
            return self.maps.get(robot_name, None)

    def battery_soc(self, robot_name: str):
        return 1.0

    def is_command_completed(self, robot_name):
        if not self._nav_done.get(robot_name, False):
            return False
        dest = self._dest.get(robot_name)
        if dest is None:
            return True
        p = self.position(robot_name)
        if p is None:
            return False
        return math.hypot(p[0] - dest[0], p[1] - dest[1]) < 0.5

    def get_data(self, robot_name: str):
        map_str = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if map_str is None or position is None or battery_soc is None:
            return None
        return RobotUpdateData(robot_name, map_str, position, battery_soc)

    def navigate(self, robot_name, pose, map_name, speed_limit=0.0):
        self._cmd_id[robot_name] = self._cmd_id.get(robot_name, 0) + 1
        cmd_id = self._cmd_id[robot_name]
        self._nav_done[robot_name] = False
        self._dest[robot_name] = (pose[0], pose[1])

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

        fut = self.nav_clients[robot_name].send_goal_async(goal_msg)
        fut.add_done_callback(
            lambda f, r=robot_name, c=cmd_id: self._goal_response_cb(r, c, f))
        return True

    def _goal_response_cb(self, robot_name, cmd_id, future):
        if cmd_id != self._cmd_id.get(robot_name):
            return                              # stale — newer command exists
        gh = future.result()
        if not gh.accepted:
            return
        self._goal_handle[robot_name] = gh
        gh.get_result_async().add_done_callback(
            lambda f, r=robot_name, c=cmd_id: self._goal_result_cb(r, c, f))

    def _goal_result_cb(self, robot_name, cmd_id, future):
        if cmd_id != self._cmd_id.get(robot_name):
            return                              # result of preempted/old goal — IGNORE
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self._nav_done[robot_name] = True
        # CANCELED/ABORTED: stays False → RMF replans on its own

    def stop(self, robot_name):
        self.get_logger().info(f"[RobotClientAPI] Processing STOP for {robot_name}")
        self._cmd_id[robot_name] = self._cmd_id.get(robot_name, 0) + 1
        self._nav_done[robot_name] = False
        gh = self._goal_handle.pop(robot_name, None)
        if gh is not None:
            self.get_logger().info(f"[RobotClientAPI] Canceling active goal for {robot_name}")
            gh.cancel_goal_async()
        return True        
        
class RobotUpdateData:
    def __init__(self, robot_name: str, map: str, position: list[float], battery_soc: float, requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position      
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
