import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_task_msgs.msg import ApiRequest
import json
import time
import uuid

# ==============================================================================
# BLOCK 1: TASK PAYLOAD GENERATOR
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


# ==============================================================================
# BLOCK 2: DISPATCHER LOGIC
# Handles the iteration, UUID generation, and ROS 2 publishing.
# ==============================================================================
def dispatch_tasks(node, publisher, tasks):
    for prefix, places in tasks:
        payload = make_patrol(places)
        msg = ApiRequest()
        msg.request_id = f"wms_{prefix}_" + str(uuid.uuid4())[:8]
        msg.json_msg = json.dumps(payload)
        publisher.publish(msg)
        
        node.get_logger().info(
            f"Dispatched {prefix}: {' → '.join(places)}"
        )
        
        # ---> FIX: Staggered Dispatch <---
        # Pause for 8 seconds before sending the next task. 
        # This gives the previous robot time to exit the room and clear the corridor.
        time.sleep(45.0)
# ==============================================================================
# BLOCK 3: MAIN NODE INITIALIZATION & EXECUTION
# ==============================================================================
def main():
    rclpy.init()
    node = rclpy.create_node('wms_task_generator')

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    pub = node.create_publisher(ApiRequest, '/task_api_requests', qos)
    
    # Wait for publisher to establish connection to the DDS network
    time.sleep(1.0)

    # Define the tasks exactly as provided
    tasks = [
        ("task1", ["pallet_pick_1",  "pallet_place_1"]),
        ("task2", ["pallet_pick_2",  "pallet_place_2"]),   # was "pick_pallet_2"
        #("task3", ["pallet_pick_3",  "pallet_place_3"]),
        #("task3", ["pallet_pick_4",  "pallet_place_4"]),
      # ("task3", ["pallet_pick_5",  "pallet_place_5"]),
    ]

    # Execute the dispatch
    dispatch_tasks(node, pub, tasks)

    node.get_logger().info(
        "All 5 tasks sent. RMF owns assignment, scheduling and traffic."
    )
    
    # Clean shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
