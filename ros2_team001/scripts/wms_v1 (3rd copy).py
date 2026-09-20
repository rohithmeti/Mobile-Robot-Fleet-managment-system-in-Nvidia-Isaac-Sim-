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

    # FIX: This fleet adapter uses the low-level RobotCommandHandle API.
    # The correct task category is "patrol", not "go_to_place".
    # "go_to_place" belongs to the newer EasyFullControl API only.
    # "patrol" with rounds=1 and a single place = go to that waypoint once.

    # hub_1 exits first
    payload_1 = {
        "type": "robot_task_request",
        "robot": "iw_hub_1",
        "fleet": "iw_hub_fleet",
        "request": {
            "category": "patrol",
            "description": {
                "places": ["docking_room_exit"],
                "rounds": 1
            }
        }
    }
    msg_1 = ApiRequest()
    msg_1.request_id = "wms_exit_hub1_" + str(uuid.uuid4())[:8]
    msg_1.json_msg = json.dumps(payload_1)
    pub.publish(msg_1)
    node.get_logger().info("Dispatched iw_hub_1 -> docking_room_exit [GOES FIRST]")

    time.sleep(0.5)

    # hub_3 waits for hub_1 to clear exit lane
    payload_3 = {
        "type": "robot_task_request",
        "robot": "iw_hub_3",
        "fleet": "iw_hub_fleet",
        "request": {
            "category": "patrol",
            "description": {
                "places": ["docking_room_exit"],
                "rounds": 1
            }
        }
    }
    msg_3 = ApiRequest()
    msg_3.request_id = "wms_exit_hub3_" + str(uuid.uuid4())[:8]
    msg_3.json_msg = json.dumps(payload_3)
    pub.publish(msg_3)
    node.get_logger().info("Dispatched iw_hub_3 -> docking_room_exit [WAITS FOR hub_1]")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
