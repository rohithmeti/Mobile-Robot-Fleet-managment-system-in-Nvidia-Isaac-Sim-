import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
import yaml

class NetworkScanner(Node):
    def __init__(self):
        super().__init__('network_scanner')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_and_save(self):
        filename = "ros2_network_dump.txt"
        with open(filename, 'w') as f:
            f.write("========================================\n")
            f.write("        ACTIVE ROS 2 TOPICS             \n")
            f.write("========================================\n")
            topic_names_and_types = self.get_topic_names_and_types()
            for name, types in sorted(topic_names_and_types):
                f.write(f"Topic: {name} | Type: {types[0]}\n")
            
            f.write("\n========================================\n")
            f.write("      LISTENING FOR TF FRAMES...        \n")
            f.write("========================================\n")
            
            print("Listening to the network for 3 seconds. Please wait...")
            
            # Properly spin the node so it can receive the TF messages
            end_time = time.time() + 3.0
            while time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)

            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            if frames_yaml == "{}" or not frames_yaml:
                f.write("CRITICAL ERROR: No TF frames detected!\n")
            else:
                try:
                    frames = yaml.safe_load(frames_yaml)
                    if frames:
                        for frame in sorted(frames.keys()):
                            f.write(f"Frame ID: '{frame}'\n")
                except Exception as e:
                    f.write(f"Error parsing frames: {e}\n")
            
            f.write("========================================\n")
            
        print(f"Done! Results saved to {filename}")

def main(args=None):
    rclpy.init(args=args)
    scanner = NetworkScanner()
    scanner.scan_and_save()
    scanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
