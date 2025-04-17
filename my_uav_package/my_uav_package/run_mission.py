import subprocess
import time
import rclpy
from rclpy.node import Node

class MissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')

    def run_script_for_duration(self, script_name, duration):
        """Runs a ROS 2 node for a specific duration, then stops it."""
        process = subprocess.Popen(["ros2", "run", "my_uav_package", script_name])
        time.sleep(duration)
        process.terminate()
        process.wait()

    def start_mission(self):
        self.get_logger().info("▶️ Starting GPS Follower...")
        self.run_script_for_duration("gps_follower", 23)

        self.get_logger().info("⏩ Switching to Flight Controller...")
        flight_process = subprocess.Popen(["ros2", "run", "my_uav_package", "flight_controller"])

        # Keep the process alive to ensure flight_controller stays running
        flight_process.wait()

def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    node.start_mission()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
