import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import numpy as np
import scipy.interpolate as interp
from pymavlink import mavutil

class ExternalFlightController(Node):
    def __init__(self):
        super().__init__('external_flight_controller')

        # MAVLink connection
        self.connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
        self.target_system = 1
        self.target_component = 1

        # Wait until heartbeat received
        print("🔄 Waiting for MAVLink heartbeat...")
        self.connection.wait_heartbeat()
        print("✅ MAVLink heartbeat received!")

        # Subscribe to AprilTag pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/apriltag/pose',
            self.pose_callback,
            10)

        ...
        # Store AprilTag position
        self.x_initial = None
        self.y_initial = None
        self.z_initial = None
        self.received_pose = False

        # Wait 23 seconds before allowing control
        self.active = False
        self.create_timer(23.0, self.activate_controller)

    def activate_controller(self):
        self.get_logger().info("🕒 23 seconds passed — external flight controller active.")
        self.active = True



    def pose_callback(self, msg):
        """ Callback function when AprilTag pose is detected. """
        if not self.active:
            return  # Not active yet

        if not self.received_pose:
            self.x_initial = msg.pose.position.x
            self.y_initial = msg.pose.position.y
            self.z_initial = msg.pose.position.z
            self.received_pose = True

            self.get_logger().info(f"📸 AprilTag Detected! x={self.x_initial}, y={self.y_initial}, z={self.z_initial}")

            # Stop following waypoint and take external control
            self.override_control()

    def override_control(self):
        """ Override ArduPilot control immediately with external commands. """
        self.following_waypoint = False

        # Step 1: Generate a minimum jerk trajectory
        wpts = np.array([[self.x_initial, 0], 
                         [0, 0], 
                         [self.z_initial, 0]])

        distance = np.sqrt(self.x_initial**2 + self.y_initial**2)
        t_end = max(distance, 0.1)  # Prevent division by zero
        tpts = np.array([0, t_end])
        numsamples = 100
        tsamples = np.linspace(0, t_end, numsamples)

        q, qd, qdd, qddd = self.min_jerk_trajectory(wpts, tpts, tsamples)

        # Step 2: Override flight with external commands
        for i in range(numsamples):
            x_vel = qd[0, i]      # Forward velocity
            y_accel = qdd[1, i]   # Lateral acceleration
            z_accel = qdd[2, i]   # Vertical acceleration

            print(f"🔹 Sending Control | X_VEL: {x_vel:.3f}, Y_ACCEL: {y_accel:.3f}, Z_ACCEL: {z_accel:.3f}")

            # === Convert to roll and pitch
            g = 9.81
            roll = np.arctan2(y_accel, g)
            alpha = np.radians(5.0)
            pitch = alpha + np.arcsin(np.clip(z_accel / g, -0.9, 0.9))

            # === Send commands
            self.send_attitude_target(roll, pitch)
            self.set_airspeed(x_vel)

            # === Timing
            if i < numsamples - 1:
                tsample_step = tsamples[i + 1] - tsamples[i]
                time.sleep(tsample_step)


        self.get_logger().info("✅ External trajectory execution complete.")

    def send_waypoint(self, lat, lon, alt):
        """ Send a waypoint for normal navigation before AprilTag detection. """
        print(f"📍 Sending waypoint: {lat}, {lon}, alt {alt}m")
        self.connection.mav.set_position_target_global_int_send(
            0, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000,  # Follow position
            int(lat * 1e7), int(lon * 1e7), float(alt),
            0, 0, 0,  # Velocity ignored
            0, 0, 0,  # Acceleration ignored
            0, 0  # Yaw ignored
        )

    def send_attitude_target(self, roll, pitch, yaw=0.0):
        """ Send roll & pitch setpoint via MAVLink attitude target. """
        def euler_to_quaternion(roll, pitch, yaw):
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            return [
                cr * cp * cy + sr * sp * sy,  # w
                sr * cp * cy - cr * sp * sy,  # x
                cr * sp * cy + sr * cp * sy,  # y
                cr * cp * sy - sr * sp * cy   # z
            ]

        q = euler_to_quaternion(roll, pitch, yaw)
        time_boot_ms = int(time.time() * 1000) % (2**32)

        self.connection.mav.set_attitude_target_send(
            time_boot_ms,
            self.target_system,
            self.target_component,
            0b10000111,  # Ignore body rates and thrust
            q,
            0, 0, 0,
            0  # thrust ignored
        )
        print(f"🎯 Attitude sent → Roll: {np.degrees(roll):.1f}°, Pitch: {np.degrees(pitch):.1f}°")


    def set_airspeed(self, speed):
        """ Set forward airspeed. """
        adjusted_speed = speed  # Adjust airspeed offset
        self.connection.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,  # Confirmation
            0,  # Speed type (0 = airspeed)
            float(adjusted_speed),
            -1, 0, 0, 0, 0
        )
        print(f"✈️ Sent airspeed: {adjusted_speed:.3f} m/s")

    def min_jerk_trajectory(self, wpts, tpts, tsamples):
        """ Compute minimum jerk trajectory. """
        q, qd, qdd, qddd = [], [], [], []

        for i in range(wpts.shape[0]):
            cs = interp.CubicSpline(tpts, wpts[i], bc_type=((1, 0), (1, 0)))
            q.append(cs(tsamples))      # Position
            qd.append(cs(tsamples, 1))  # Velocity
            qdd.append(cs(tsamples, 2)) # Acceleration
            qddd.append(cs(tsamples, 3))# Jerk
        
        return np.array(q), np.array(qd), np.array(qdd), np.array(qddd)

def main(args=None):
    rclpy.init(args=args)
    node = ExternalFlightController()

    try:
        rclpy.spin(node)  # Keep node running
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down External Flight Controller...")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
