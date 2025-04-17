import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # ✅ New import to publish GPS active status

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_dronekit')

        # ✅ Connect to ArduPilot SITL using DroneKit
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        # ✅ ROS 2 Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.gps_active_pub = self.create_publisher(Bool, '/gps_following_active', 10)  # ✅ Publish GPS status

        # ✅ ROS 2 Subscriber
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        # ✅ Timing & Control
        self.gps_start_time = None
        self.following_active = False
        self.fixed_altitude = 10.0  # UAV flies at 10m
        self.last_lat, self.last_lon = None, None  # Smoothing for GPS
        self.pbvs_switch_time = 23  # ✅ Switch to PBVS after 7 seconds

        # ✅ Take off UAV
        self.takeoff(self.fixed_altitude)

        # ✅ Start Prius movement
        time.sleep(2)  # Small delay before Prius moves
        self.start_prius_movement()

    def send_gps_target(self, lat, lon):
        """ Commands UAV to move to target GPS coordinates. """
        if not self.following_active:
            return

        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon
        self.last_lat, self.last_lon = lat, lon

        if self.vehicle.mode != VehicleMode("GUIDED"):
            self.get_logger().warn("Switching UAV to GUIDED mode.")
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)

        target_location = LocationGlobalRelative(lat, lon, self.fixed_altitude)
        self.get_logger().info(f"Moving UAV to: {lat}, {lon}, Alt: {self.fixed_altitude}m")
        self.vehicle.simple_goto(target_location)
        time.sleep(0.5)  # Small delay to allow UAV to process the command

    def gps_callback(self, msg):
        """ Processes Prius GPS data and starts PBVS after 7 seconds. """
        lat, lon = msg.latitude, msg.longitude

        if self.gps_start_time is None:
            self.gps_start_time = time.time()
            self.following_active = True
            self.get_logger().info("GPS Following started!")

        # ✅ Publish GPS active status
        gps_msg = Bool()
        if time.time() - self.gps_start_time < self.pbvs_switch_time:
            gps_msg.data = True  # GPS mode active
            self.send_gps_target(lat, lon)  # Follow GPS
        else:
            gps_msg.data = False  # ✅ Stop GPS following after 7 seconds
            self.following_active = False
            self.get_logger().info("✅ GPS Following stopped. Switching to PBVS.")

        self.gps_active_pub.publish(gps_msg)

    def takeoff(self, target_altitude):
        """ Arms and takes off UAV to the specified altitude. """
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
            self.get_logger().info(f"Altitude: {altitude:.1f}m")
            if altitude >= target_altitude * 0.95:
                self.get_logger().info("Reached target altitude!")
                break
            time.sleep(1)

    def start_prius_movement(self):
        """ Moves Prius at 16 m/s. """
        vel_msg = Twist()
        vel_msg.linear.x = 16.0  # Move forward at 16 m/s
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("Prius moving forward at 16 m/s...")

def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()
    rclpy.spin(gps_follower)
    gps_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

