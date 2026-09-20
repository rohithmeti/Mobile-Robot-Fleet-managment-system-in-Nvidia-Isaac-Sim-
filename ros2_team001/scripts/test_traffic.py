import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_task_msgs.msg import ApiRequest
import json
import time
import uuid

def main():
    rclpy.init()
    node = rclpy.create_node('wms_task_generator')

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    pub = node.create_publisher(ApiRequest, '/task_api_requests', qos)
    time.sleep(1.0)

    current_sim_time_ms = int(node.get_clock().now().nanoseconds / 1000000)

    # --- TASK 1: Pure Open Auction ---
    payload = {
        "type": "dispatch_task_request",
        "request": {
            "unix_millis_earliest_start_time": current_sim_time_ms,
            "category": "patrol",
            "description": {
                "places": ["docking_room_exit"],
                "rounds": 1
            }
        }
    }
    
    msg = ApiRequest()
    msg.request_id = "wms_auction_" + str(uuid.uuid4())[:8]
    msg.json_msg = json.dumps(payload)
    
    pub.publish(msg)
    node.get_logger().info("Dispatched 1 Open Auction Task to the network.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
