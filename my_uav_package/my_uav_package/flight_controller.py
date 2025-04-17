import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import numpy as np
from pymavlink import mavutil
import pandas as pd

class ExternalFlightController(Node):
    def __init__(self):
        super().__init__('external_flight_controller')

        # MAVLink connection
        self.connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")
        self.target_system = 1
        self.target_component = 1

        # Car speed (m/s)
        self.car_speed = 16.0
        self.current_airspeed = 0.0  # To store real-time airspeed

        # Wait until heartbeat received
        print("🔄 Waiting for MAVLink heartbeat...")
        self.connection.wait_heartbeat()
        print("✅ MAVLink heartbeat received!")

        # Start listening for airspeed updates
        self.get_airspeed()

        # Subscribe to AprilTag pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/apriltag/pose',
            self.pose_callback,
            10)

        # Store AprilTag position
        self.x_initial = None
        self.y_initial = None
        self.z_initial = None
        self.received_pose = False  # Flag to check if data is received

    def get_airspeed(self):
        """ Continuously update the UAV's real-time airspeed from MAVLink. """
        def update_airspeed():
            while True:
                msg = self.connection.recv_match(type='VFR_HUD', blocking=True)
                if msg:
                    self.current_airspeed = msg.airspeed  # m/s
                    print(f"📡 Current Airspeed: {self.current_airspeed:.3f} m/s")
        
        # Start airspeed listener in a separate thread
        import threading
        airspeed_thread = threading.Thread(target=update_airspeed, daemon=True)
        airspeed_thread.start()

    def pose_callback(self, msg):
        """ Callback function when AprilTag pose is detected. """
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

        # Generate adaptive landing trajectory
        trajectory_df = self.generate_adaptive_landing_trajectory(self.x_initial, self.y_initial, self.z_initial, duration=5.0, time_step=0.1)

        start_time = time.time()
        for i, row in trajectory_df.iterrows():
            x_vel = row['vx']
            y_accel = row['ay']
            z_vel = row['vz']
            expected_time = row['time']

            airspeed_diff = x_vel - self.current_airspeed
            print(f"🔹 Commanded Airspeed: {x_vel:.3f} m/s | Current Airspeed: {self.current_airspeed:.3f} m/s | Diff: {airspeed_diff:.3f} m/s")

            self.send_lateral_acceleration(y_accel, enable_external=1)
            self.set_airspeed(x_vel)
            self.send_climb_rate(z_vel, enable_external=1)

            # Synchronize with trajectory timestamps
            elapsed_time = time.time() - start_time
            sleep_time = max(0, expected_time - elapsed_time)
            time.sleep(sleep_time)

        self.get_logger().info("✅ External trajectory execution complete.")

    def generate_adaptive_landing_trajectory(self, x, y, z, duration=5.0, time_step=0.1):
        """ Generate a trajectory that first stabilizes vx & vy before descending in vz. """
        t = np.arange(0, duration + time_step, time_step)
        T = duration / 2  # Split into two phases

        # Phase 1: Speed Matching & Lateral Stabilization
        s1 = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
        s1_dot = (30 * (t/T)**2 - 60 * (t/T)**3 + 30 * (t/T)**4) / T
        s1_ddot = (60 * (t/T) - 180 * (t/T)**2 + 120 * (t/T)**3) / (T**2)

        x_traj = self.car_speed * t  # Match car speed
        y_traj = y * (1 - s1)  # Stabilize lateral position
        z_traj = np.full_like(t, z)  # Hold altitude in Phase 1

        x_vel = np.full_like(t, self.car_speed)  # Maintain constant car speed
        y_vel = -y * s1_dot  # Reduce lateral velocity
        z_vel = np.zeros_like(t)  # No descent yet

        x_accel = np.zeros_like(t)  # No acceleration in x
        y_accel = -y * s1_ddot  # Lateral acceleration
        z_accel = np.zeros_like(t)  # No vertical acceleration yet

        # Phase 2: Start Descent After vx & vy are near zero
        descent_start = int(len(t) / 2)
        s2 = 10 * ((t - T)/T)**3 - 15 * ((t - T)/T)**4 + 6 * ((t - T)/T)**5
        s2 = np.clip(s2, 0, 1)

        z_traj[descent_start:] = z * (1 - s2[descent_start:])
        z_vel[descent_start:] = -z * s2[descent_start:] / T
        z_accel[descent_start:] = -z * (s2[descent_start:] - np.roll(s2, 1)[descent_start:]) / (T**2)

        return pd.DataFrame({
            'time': t,
            'x': x_traj, 'y': y_traj, 'z': z_traj,
            'vx': x_vel, 'vy': y_vel, 'vz': z_vel,
            'ax': x_accel, 'ay': y_accel, 'az': z_accel
        })


def main(args=None):
    rclpy.init(args=args)
    node = ExternalFlightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down External Flight Controller...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
