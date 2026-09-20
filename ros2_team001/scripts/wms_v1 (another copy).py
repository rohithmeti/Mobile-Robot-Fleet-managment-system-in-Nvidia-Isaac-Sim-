import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_task_msgs.msg import ApiRequest
import json
import time
import uuid

# ==============================================================================
# WMS v2 — 3 Pick→Place patrol tasks, fully RMF-managed
# RMF handles: robot selection, scheduling, traffic, negotiation
# WMS only sends task descriptions — no robot names, no delays
# ==============================================================================

def make_patrol(places, rounds=1):
    return {
        "type": "dispatch_task_request",
        "request": {
            "category": "patrol",
            "description": {
                "places": places,
                "rounds": rounds
            },
            "requester": "wms"
        }
    }

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

    tasks = [
        # (task_id_prefix, places)
        
        ("task1", ["pick_pallet_1", "pallet_place_1"]),
        ("task2", ["pick_pallet_2", "pallet_place_2"]),
        ("task3", ["pick_pallet_3", "pallet_place_3"]),
        
    ]

    for prefix, places in tasks:
        payload = make_patrol(places)
        msg = ApiRequest()
        msg.request_id = f"wms_{prefix}_" + str(uuid.uuid4())[:8]
        msg.json_msg = json.dumps(payload)
        pub.publish(msg)
        node.get_logger().info(
            f"Dispatched {prefix}: {' → '.join(places)}"
        )

    node.get_logger().info(
        "All 3 tasks sent. RMF owns assignment, scheduling and traffic."
    )
    rclpy.shutdown()

if __name__ == '__main__':
    main()
