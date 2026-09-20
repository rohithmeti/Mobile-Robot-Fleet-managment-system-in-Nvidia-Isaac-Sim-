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
        
        self.positions = {}          
        self.covariances = {}        
        self.maps = {}               
        self.nav_clients = {}        
        self.initial_pose_pubs = {}  
        self.command_completed = {}  
        self.goal_handles = {}       

        for name in self.robot_names:
            self.positions[name] = None   
            self.covariances[name] = 0.0  
            self.maps[name] = robots_dict.get(name, {}).get('initial_map', 'L1')
            self.command_completed[name] = True

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

    def is_command_completed(self, robot_name: str) -> bool:
        with self._lock:
            return self.command_completed.get(robot_name, False)

    def get_data(self, robot_name: str):
        map_str = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if map_str is None or position is None or battery_soc is None:
            return None
        return RobotUpdateData(robot_name, map_str, position, battery_soc)

    def navigate(self, robot_name: str, pose, map_name: str, speed_limit=0.0):
        if robot_name not in self.nav_clients:
            return False
        
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
        server_ready = False
        for _ in range(10):
            if client.server_is_ready():
                server_ready = True
                break
            time.sleep(0.5)
            
        if not server_ready:
            with self._lock:
                self.command_completed[robot_name] = True
            return False
            
        send_goal_future = client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future, n=robot_name: self._goal_response_callback(future, n))
        return True

    def _goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            with self._lock:
                self.command_completed[robot_name] = True
            return
        with self._lock:
            self.goal_handles[robot_name] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future, n=robot_name: self._goal_result_callback(future, n))

    def _goal_result_callback(self, future, robot_name):
        with self._lock:
            self.command_completed[robot_name] = True

    def stop(self, robot_name: str):
        with self._lock:
            handle = self.goal_handles.get(robot_name)
        if handle is not None:
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda future, n=robot_name: self._cancel_done_callback(future, n))
        return True

    def _cancel_done_callback(self, future, robot_name):
        pass

class RobotUpdateData:
    def __init__(self, robot_name: str, map: str, position: list[float], battery_soc: float, requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position      
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
