import sys
import argparse
import yaml
import time
import threading
import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy

from .RobotClientAPI import RobotAPI

# ==============================================================================
# BLOCK 1: EASY FULL CONTROL ROBOT ADAPTER
# ==============================================================================
class RobotAdapter:
    def __init__(self, name, api, node):
        self.name = name
        self.api = api
        self.node = node
        self.execution = None
        self.update_handle = None

    def navigate(self, destination, execution):
        """Called by RMF when the robot is granted traffic clearance to move."""
        self.node.get_logger().info(f"[{self.name}] Clearance granted. Navigating to: {destination.position}")
        self.execution = execution
        self.api.navigate(self.name, destination.position, destination.map, destination.speed_limit)

    def stop(self, activity):
        """Called by RMF to halt the robot immediately for crossing traffic."""
        self.node.get_logger().info(f"[{self.name}] Traffic override. Stopping immediately.")
        self.api.stop(self.name)
        if self.execution is not None:
            self.execution = None

    def execute_action(self, category, description, execution):
        """Placeholder for custom payload actions."""
        self.node.get_logger().info(f"[{self.name}] Action {category} requested.")
        self.execution = execution
        self.execution.finished()

    def update(self):
        """Called constantly by the update_loop to feed physical odometry to RMF."""
        data = self.api.get_data(self.name)
        if data is None:
            return

        # AMCL Covariance Safety Check
        cov = self.api.pose_covariance(self.name)
        if cov == 0.0 or cov > 0.5:
            return

        # If Nav2 reached the destination, tell RMF we are done with this leg
        if self.execution is not None and self.api.is_command_completed(self.name):
            self.node.get_logger().info(f"[{self.name}] Leg finished.")
            self.execution.finished()
            self.execution = None

        # Push the exact physical state up to the RMF traffic scheduler
        state = rmf_easy.RobotState(
            data.map,
            data.position,
            1.0 # Hardcoded 100% Battery to prevent freezes
        )
        if self.update_handle is not None:
            self.update_handle.update(state, self.api.is_command_completed(self.name))


# ==============================================================================
# BLOCK 2: MAIN ENTRY & CONFIGURATION
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
    node = rclpy.node.Node(f'{fleet_name}_easy_adapter')
    adapter = Adapter.make(f'{fleet_name}_easy_adapter')

    if args.use_sim_time:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        adapter.node.use_sim_time()

    api = RobotAPI(config_yaml=config_yaml, use_sim_time=args.use_sim_time)

# ==============================================================================
# BLOCK 3: FLEET INITIALIZATION & ROBOT REGISTRATION
# ==============================================================================
    # Use the official EasyFullControl parser
    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        args.config_file, args.nav_graph
    )
    fleet_handle = adapter.add_easy_fleet(fleet_config)

    robot_adapters = {}
    for robot_name in config_yaml['rmf_fleet']['robots']:
        rob_adapter = RobotAdapter(robot_name, api, node)
        
        # Bind the functions strictly to RMF Callbacks
        callbacks = rmf_easy.RobotCallbacks(
            rob_adapter.navigate,
            rob_adapter.stop,
            rob_adapter.execute_action
        )
        
        data = api.get_data(robot_name)
        while data is None or api.pose_covariance(robot_name) == 0.0:
            node.get_logger().info(f"Waiting for converged AMCL pose for {robot_name}...")
            time.sleep(1.0)
            data = api.get_data(robot_name)

        initial_state = rmf_easy.RobotState(data.map, data.position, 1.0)
        
        # Register to the fleet
        update_handle = fleet_handle.add_robot(
            robot_name,
            initial_state,
            callbacks
        )
        rob_adapter.update_handle = update_handle
        robot_adapters[robot_name] = rob_adapter

    adapter.start()

# ==============================================================================
# BLOCK 4: HIGH-FREQUENCY UPDATE LOOP
# ==============================================================================
    def update_loop():
        freq = config_yaml['rmf_fleet'].get('robot_state_update_frequency', 20.0)
        sleep_time = 1.0 / freq
        while rclpy.ok():
            for rob in robot_adapters.values():
                try:
                    rob.update()
                except Exception as e:
                    node.get_logger().error(f"Error updating {rob.name}: {e}")
            time.sleep(sleep_time)

    threading.Thread(target=update_loop, daemon=True).start()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
