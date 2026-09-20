import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math
import time
from datetime import datetime

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')
        
        # Create a unique file for this run
        self.file_name = f"amr_pose_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        self.get_logger().info(f"Logging AMR data to: {self.file_name}")
        
        # Write the header
        with open(self.file_name, 'w') as f:
            f.write("TIMESTAMP | ROBOT | TOPIC | X | Y | YAW(deg)\n")
            f.write("-" * 65 + "\n")
            
        # Dictionary to throttle logging (prevents massive file sizes)
        self.last_log_time = {}
        self.log_interval = 1.0 # Only log once per second per topic per robot

        for i in range(1, 6):
            robot_name = f'iw_hub_{i}'
            self.last_log_time[robot_name] = {'amcl': 0.0, 'goal': 0.0}
            
            # Subscribe to AMCL
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{robot_name}/amcl_pose',
                lambda msg, name=robot_name: self.amcl_callback(msg, name),
                10
            )
            
            # Subscribe to Goal Pose
            self.create_subscription(
                PoseStamped,
                f'/{robot_name}/goal_pose',
                lambda msg, name=robot_name: self.goal_callback(msg, name),
                10
            )

    def euler_from_quaternion(self, x, y, z, w):
        """Converts quaternion to euler yaw (in degrees) for easy reading"""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(t3, t4)
        return math.degrees(yaw_rad)

    def write_to_file(self, robot_name, topic_type, x, y, yaw):
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        log_string = f"{timestamp} | {robot_name} | {topic_type} | X: {x:.2f} | Y: {y:.2f} | Yaw: {yaw:.2f}°\n"
        
        with open(self.file_name, 'a') as f:
            f.write(log_string)
            
        # Print to terminal so you know it's working
        self.get_logger().info(log_string.strip())

    def amcl_callback(self, msg, robot_name):
        current_time = time.time()
        if current_time - self.last_log_time[robot_name]['amcl'] >= self.log_interval:
            self.last_log_time[robot_name]['amcl'] = current_time
            
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            yaw = self.euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
            
            self.write_to_file(robot_name, "AMCL", pos.x, pos.y, yaw)

    def goal_callback(self, msg, robot_name):
        current_time = time.time()
        if current_time - self.last_log_time[robot_name]['goal'] >= self.log_interval:
            self.last_log_time[robot_name]['goal'] = current_time
            
            pos = msg.pose.position
            ori = msg.pose.orientation
            yaw = self.euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
            
            self.write_to_file(robot_name, "GOAL", pos.x, pos.y, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Logging stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
