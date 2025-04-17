import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseStamped
import math

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_to_landing_controller')

        # Connect to ArduPilot via DroneKit
        self.get_logger().info("Connecting to UAV...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("Connected to UAV!")

        # ROS 2 publishers/subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 5)
        self.create_subscription(NavSatFix, '/prius/gps', self.gps_callback, 10)
        self.create_subscription(PoseStamped, '/apriltag/pose', self.pose_callback, 10)

        # State
        self.fixed_altitude = 10.0           # Starting altitude for takeoff + GPS follow
        self.current_z = self.fixed_altitude # Updated once landing guidance starts
        self.following = False
        self.landing_active = False
        self.init_pose_recorded = False
        self.x0 = None
        self.y0 = None
        self.k = None
        self.kv = None
        self.vmin = 15.0
        self.v0 = 20.0

        # Speed used during GPS follow
        self.gps_follow_speed = 18.0  # e.g. slightly above 16 m/s so UAV can catch up

        # Start sequence
        self.takeoff(self.fixed_altitude)
        time.sleep(2)
        self.start_prius_movement()
        # Immediately schedule the timer to start GPS follow and another to switch to landing
        self.timer = self.create_timer(0.0, self.start_following)
        self.shutdown_timer = self.create_timer(23.0, self.activate_landing)

    def takeoff(self, target_altitude):
        """Arms and take off to 'target_altitude' meters"""
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            self.get_logger().info("Waiting for UAV to arm...")
            time.sleep(1)
        self.get_logger().info(f"Taking off to {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                self.get_logger().info(f"Reached takeoff altitude: {alt:.2f}m")
                break
            time.sleep(1)

    def start_prius_movement(self):
        """Command the Prius to move forward at 16 m/s"""
        vel_msg = Twist()
        vel_msg.linear.x = 16.0
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info("Prius moving forward at 16 m/s.")

    def start_following(self):
        """Enables GPS following using the Prius lat/lon"""
        self.following = True
        self.destroy_timer(self.timer)
        self.get_logger().info("✅ UAV is now following Prius via GPS")

    def gps_callback(self, msg: NavSatFix):
        """Continuously updates the UAV's waypoint to the Prius's lat/lon with altitude 'current_z', and sets speed"""
        lat, lon = msg.latitude, msg.longitude

        # If we haven't begun following, do nothing
        # If we have, we do a simple_goto at altitude
        self.vehicle.mode = VehicleMode("GUIDED")
        altitude = self.fixed_altitude if not self.landing_active else self.current_z
        target_location = LocationGlobalRelative(lat, lon, altitude)
        self.vehicle.simple_goto(target_location)

        # Send DO_CHANGE_SPEED if still in GPS following
        if not self.landing_active:
            speed_msg = self.vehicle.message_factory.command_long_encode(
                0, 0,   # target system, component
                178,    # MAV_CMD_DO_CHANGE_SPEED
                0,      # confirmation
                1,      # speed type (airspeed)
                self.gps_follow_speed,
                -1, 0, 0, 0, 0
            )
            self.vehicle.send_mavlink(speed_msg)

        # Slight delay to reduce spam
        time.sleep(0.5)

    def activate_landing(self):
        """Stops GPS-based altitude and transitions to AprilTag-based landing guidance"""
        self.get_logger().info("🛬 Switching to AprilTag-based landing mode")
        self.following = False
        self.landing_active = True

    def pose_callback(self, msg: PoseStamped):
        """
        If in landing mode, compute desired altitude offset from the paper's exponential guidance
        and update the UAV's altitude while still sending lat/lon from the GPS.
        """
        # If not in landing phase, do nothing
        if not self.landing_active:
            return

        # x_rel > 0 means the tag is in front. z_rel < 0 means tag is below UAV.
        x_rel = msg.pose.position.x
        z_rel = -msg.pose.position.z  # Because your AprilTag node outputs +Z = below UAV

        # On first detection, record x0, y0, and compute k, kv
        if not self.init_pose_recorded:
            if x_rel <= 0 or z_rel >= 0:
                self.get_logger().warn("❌ Invalid initial tag pose. Make sure UAV is behind and above the car.")
                return
            self.x0 = x_rel
            self.y0 = z_rel  # negative
            try:
                # from the paper
                self.k = (0.01 - self.x0) / -math.log(0.01 / -self.y0)
                self.kv = self.x0 / -math.log((2 - self.v0 / self.vmin))
            except ValueError:
                self.get_logger().warn("Invalid guidance params.")
                return
            self.init_pose_recorded = True
            self.get_logger().info("[Landing Init] Recorded x0,y0 => Exponential guidance begun.")
            return

        # If we're close to the tag, hold.
        if abs(z_rel) < 1.0:
            self.get_logger().info("✅ Close enough to landing target. Holding descent.")
            return

        # 1) Compute desired altitude offset from the paper
        try:
            desired_z = self.y0 * (1 - math.exp(-(x_rel - self.x0) / self.k))
        except OverflowError:
            desired_z = self.y0

        # 2) Cap it at 0.0 so we never ascend above the original altitude
        desired_z = min(0.0, desired_z)

        # 3) Compute horizontal speed from eqn. (4)
        desired_vx = self.vmin * (2 - math.exp(-x_rel / self.kv))
        # clamp between vmin and v0
        desired_vx = max(self.vmin, min(desired_vx, self.v0))

        # 4) Update current altitude => 10 + negative offset => <= 10
        potential_alt = self.fixed_altitude + desired_z
        # keep it above 0.5m so we don't crash
        self.current_z = max(0.5, potential_alt)

        # Debug print
        self.get_logger().info(
            f"[Landing] x_rel={x_rel:.2f}, z_rel={z_rel:.2f}, desired_z={desired_z:.2f}, "
            f"alt={self.current_z:.2f}, vx={desired_vx:.2f}"
        )

        # 5) Send MAV_CMD_DO_CHANGE_SPEED (airspeed)
        msg1 = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, component
            178,     # MAV_CMD_DO_CHANGE_SPEED
            0,       # confirmation
            1,       # speed type (airspeed)
            desired_vx,
            -1, 0, 0, 0, 0
        )
        self.vehicle.send_mavlink(msg1)

        # NOTE: We no longer send set_position_target_local_ned
        # because we just rely on simple_goto() from gps_callback

    def shutdown_node(self):
        """Stop everything and shut down."""
        self.get_logger().info("🛑 Shutting down node")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GpsFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
