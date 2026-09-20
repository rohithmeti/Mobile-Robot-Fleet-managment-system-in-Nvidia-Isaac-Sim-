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

    # FIX: Use "dispatch_task_request" (bidding path) NOT "robot_task_request".
    # "robot_task_request" with a robot name = direct task path.
    # The direct task path is a C++ internal path in rmf_fleet_adapter that
    # does NOT call any Python callback (consider_patrol_requests etc).
    # It silently drops because Pybind11 does not expose accept_direct_requests.
    #
    # "dispatch_task_request" = bidding path.
    # Goes through consider_patrol_requests -> bid_notice -> bid_response -> dispatch.
    # RMF assigns the best available robot automatically.
    # To force hub_1 first: dispatch hub_1 task first with earliest_start_time=0,
    # then hub_3 slightly later. RMF traffic manager sequences them on shared lane.

    # Task 1: send hub_1 to docking_room_exit
    payload_1 = {
        "type": "dispatch_task_request",
        "request": {
            "category": "patrol",
            "description": {
                "places": ["docking_room_exit"],
                "rounds": 1
            },
            "requester": "wms",
            "fleet_name": "iw_hub_fleet",
            "robot_name": "iw_hub_1"
        }
    }
    msg_1 = ApiRequest()
    msg_1.request_id = "wms_exit_hub1_" + str(uuid.uuid4())[:8]
    msg_1.json_msg = json.dumps(payload_1)
    pub.publish(msg_1)
    node.get_logger().info("Dispatched iw_hub_1 -> docking_room_exit [BIDDING, GOES FIRST]")

    time.sleep(0.5)

    # Task 2: send hub_3 to docking_room_exit
    payload_3 = {
        "type": "dispatch_task_request",
        "request": {
            "category": "patrol",
            "description": {
                "places": ["docking_room_exit"],
                "rounds": 1
            },
            "requester": "wms",
            "fleet_name": "iw_hub_fleet",
            "robot_name": "iw_hub_3"
        }
    }
    msg_3 = ApiRequest()
    msg_3.request_id = "wms_exit_hub3_" + str(uuid.uuid4())[:8]
    msg_3.json_msg = json.dumps(payload_3)
    pub.publish(msg_3)
    node.get_logger().info("single amr patrol task to point [docking_room_exit] ")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
