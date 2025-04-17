import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import numpy as np

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_dronekit')

        # Connect to ArduPilot SITL using DroneKit
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        # ROS 2 Publisher to control Prius velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)

        # ROS 2 Subscriber to Prius GPS topic
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)

        # ✅ Fixed target altitude
        self.fixed_altitude = 10.0
        self.last_gps_time = None
        self.last_lat = None
        self.last_lon = None
        self.car_speed = 0.0  # Estimated car speed

        # ✅ Start UAV takeoff
        self.takeoff(self.fixed_altitude)

        # ✅ Start the Prius movement
        time.sleep(2)  
        self.start_prius_movement()

        # ✅ Start the UAV following after a delay
        self.timer = self.create_timer(0.0, self.start_following)

        # ✅ Schedule shutdown after 23 seconds
        self.shutdown_timer = self.create_timer(23.0, self.shutdown_node)

    def start_following(self):
        """ Start UAV following the Prius """
        self.get_logger().info("✅ UAV is now following the Prius.")
        self.destroy_timer(self.timer)

    def gps_callback(self, msg):
        """ Callback when new GPS data is received from Prius """
        lat, lon = msg.latitude, msg.longitude
        current_time = self.get_clock().now().to_msg().sec

        # Compute car velocity if previous GPS exists
        if self.last_gps_time is not None:
            dt = current_time - self.last_gps_time
            if dt > 0:
                distance = self.haversine(self.last_lat, self.last_lon, lat, lon)
                self.car_speed = distance / dt  # m/s
                self.get_logger().info(f"🚗 Estimated car speed: {self.car_speed:.2f} m/s")

        # Update last GPS values
        self.last_gps_time = current_time
        self.last_lat = lat
        self.last_lon = lon

        # Command UAV to follow with speed matching
        self.follow_with_speed(lat, lon, self.car_speed)
        time.sleep(0.5)

    def haversine(self, lat1, lon1, lat2, lon2):
        """ Calculate great-circle distance """
        R = 6371000  # Earth radius in meters
        phi1, phi2 = np.radians(lat1), np.radians(lat2)
        delta_phi, delta_lambda = np.radians(lat2 - lat1), np.radians(lon2 - lon1)
        a = np.sin(delta_phi/2.0)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda/2.0)**2
        return 2 * R * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

    def follow_with_speed(self, lat, lon, target_speed):
        """ Command UAV to move towards the target GPS while matching speed """
        if self.vehicle.mode != VehicleMode("GUIDED"):
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)

        target_location = LocationGlobalRelative(lat, lon, self.fixed_altitude)
        self.vehicle.simple_goto(target_location, airspeed=target_speed)
        self.get_logger().info(f"✈️ UAV moving to {lat}, {lon} at {target_speed:.2f} m/s")
        time.sleep(0.5)

    def takeoff(self, target_altitude):
        """ Arms and takes off the UAV """
        self.get_logger().info("🔹 Arming UAV...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            self.get_logger().info("Waiting for UAV to arm...")
            time.sleep(1)

        self.get_logger().info(f"🚀 Taking off to {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            if altitude >= target_altitude * 0.95:
                self.get_logger().info("✅ Reached target altitude!")
                break
            time.sleep(1)

    def start_prius_movement(self):
        """ Sends a command to move the Prius forward """
        vel_msg = Twist()
        vel_msg.linear.x = 16.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("🚗 Prius moving forward at 16 m/s...")

    def shutdown_node(self):
        """Safely shutdown the GPS follower node."""
        self.get_logger().info("🛑 Shutting down GPS follower...")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()
    rclpy.spin(gps_follower)
    gps_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()