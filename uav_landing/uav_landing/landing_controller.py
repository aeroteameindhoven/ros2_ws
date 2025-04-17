import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseStamped
from pymavlink import mavutil
import numpy as np

class UAVLandingController(Node):
    def __init__(self):
        super().__init__('uav_landing_controller')

        # ✅ Connect to UAV
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        # ✅ MAVLink connection
        self.mavlink_conn = mavutil.mavlink_connection("udp:127.0.0.1:14550")

        # ✅ ROS 2 Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)
        self.create_subscription(PoseStamped, '/apriltag/pose', self.apriltag_callback, 10)

        # ✅ Parameters
        self.fixed_altitude = 10.0
        self.speed_matching_duration = 23.0
        self.start_time = time.time()

        # ✅ Flags
        self.following_started = False
        self.using_landing_controller = False

        # ✅ Initialize last known GPS position
        self.last_lat, self.last_lon = None, None

        # ✅ Takeoff & start Prius movement
        self.takeoff(self.fixed_altitude)
        time.sleep(2)
        self.start_prius_movement()

        # ✅ Start the control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        """Main loop: GPS speed match for 23s, then switch to landing control."""
        elapsed_time = time.time() - self.start_time

        if elapsed_time < self.speed_matching_duration:
            self.following_started = True
        else:
            if not self.using_landing_controller:
                self.get_logger().info("Switching to landing controller!")
                self.using_landing_controller = True

        if self.using_landing_controller:
            self.run_landing_controller()
        else:
            self.run_gps_follower()

    def run_gps_follower(self):
        """Follows UGV using GPS speed matching."""
        if self.last_lat is None:
            return
        target_location = LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude)
        self.vehicle.simple_goto(target_location)
        time.sleep(0.5)

    def run_landing_controller(self):
        """Uses AprilTag for precision landing."""
        if self.x_uav is None:
            return
        V_cmd = self.compute_airspeed_command()
        self.send_do_change_speed(V_cmd)

    def apriltag_callback(self, msg):
        """Updates UAV pose from AprilTag."""
        self.x_uav = -msg.pose.position.x
        self.h_uav = msg.pose.position.z

    def gps_callback(self, msg):
        """Updates GPS position while running both controllers."""
        lat, lon = msg.latitude, msg.longitude
        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon
        self.last_lat, self.last_lon = lat, lon

    def compute_airspeed_command(self):
        """Computes speed based on relative position."""
        position_error = -self.x_uav
        V_cmd = 21.0 + 0.05 * position_error
        return max(V_cmd, 16.0)

    def send_do_change_speed(self, V_cmd):
        """Sends MAVLink DO_CHANGE_SPEED."""
        self.mavlink_conn.mav.command_long_send(
            self.mavlink_conn.target_system,
            self.mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0, 0, V_cmd, -1, 0, 0, 0, 0
        )

    def takeoff(self, target_altitude):
        """Takes off the UAV to the given altitude."""
        self.get_logger().info("Arming UAV...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            self.get_logger().info("Waiting for UAV to arm...")
            time.sleep(1)

        self.get_logger().info(f"Taking off to {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            if altitude >= target_altitude * 0.95:
                self.get_logger().info("Reached target altitude!")
                break
            time.sleep(1)

    def start_prius_movement(self):
        """Moves the Prius forward at 16 m/s."""
        vel_msg = Twist()
        vel_msg.linear.x = 16.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("Prius moving forward at 16 m/s...")

def main(args=None):
    rclpy.init(args=args)
    node = UAVLandingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


