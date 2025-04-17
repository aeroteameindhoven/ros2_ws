import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseStamped
from pymavlink import mavutil
import numpy as np

class UAVLandingController(Node):
    def __init__(self):
        super().__init__('gps_follower_with_landing')

        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        self.mavlink_conn = mavutil.mavlink_connection(
            "udp:127.0.0.1:14550",
            source_system=17,     # This MUST match FOLL_SYSID on ArduPilot
            source_component=1
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)

        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)
        self.create_subscription(PoseStamped, '/apriltag/pose', self.apriltag_callback, 10)

        self.fixed_altitude = 10.0
        self.speed_matching_duration = 23.0
        self.start_time = time.time()

        self.following_started = False
        self.using_landing_controller = False

        self.last_lat = None
        self.last_lon = None
        self.x_uav = None
        self.h_uav = None

        self.takeoff(self.fixed_altitude)
        time.sleep(2)
        self.start_prius_movement()

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
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
        """Send beacon position to ArduPilot for Follow Mode."""
        if self.last_lat is None or self.last_lon is None:
            return

        self.mavlink_conn.mav.set_position_target_global_int_send(
            int(time.time() * 1000),  # time_boot_ms in ms
            1,                        # target_system (ArduPilot)
            0,                        # target_component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,       # Only lat/lon/alt fields are valid
            int(self.last_lat * 1e7),  # lat (scaled)
            int(self.last_lon * 1e7),  # lon (scaled)
            self.fixed_altitude,       # alt
            0, 0, 0,                   # velocity (optional)
            0, 0, 0,                   # accel (optional)
            0, 0                       # yaw, yaw_rate
        )

    def run_landing_controller(self):
        if self.x_uav is None:
            return

        V_cmd = self.compute_airspeed_command()
        self.send_do_change_speed(V_cmd)

    def gps_callback(self, msg):
        lat, lon = msg.latitude, msg.longitude
        if self.last_lat is not None:
            lat = 0.8 * self.last_lat + 0.2 * lat
            lon = 0.8 * self.last_lon + 0.2 * lon

        self.last_lat, self.last_lon = lat, lon

    def apriltag_callback(self, msg):
        self.x_uav = -msg.pose.position.x
        self.h_uav = msg.pose.position.z

    def compute_airspeed_command(self):
        position_error = -self.x_uav
        V_cmd = 21.0 + 0.05 * position_error
        return max(V_cmd, 16.0)

    def send_do_change_speed(self, V_cmd):
        self.mavlink_conn.mav.command_long_send(
            self.mavlink_conn.target_system,
            self.mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            0,
            V_cmd,
            -1,
            0, 0, 0, 0
        )

    def takeoff(self, target_altitude):
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
