import time
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from pymavlink import mavutil
from geopy.distance import geodesic
import math
import threading
import matplotlib.pyplot as plt
import csv
import os
from std_msgs.msg import String  # At the top of your file
import pandas as pd
from apriltag_interfaces.msg import TagPoseStamped  # Your custom message

# --- PID Controller ---
class PID:
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit

    def update(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# --- Quaternion Helper ---
def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    ]

class GpsFollower(Node):
    def __init__(self):
        super().__init__('gps_follower_dronekit')


        self.create_subscription(String, 'uav1/location', self.location_callback, 10)
        self.create_subscription(TagPoseStamped, '/apriltag/pose_in_base', self.apriltag_callback, 10)
        self.serial_path = 'udpout:127.0.1:14550'
        self.connect_vehicle()

        self.last_apriltag_time = 0.0
        self.latest_apriltag_pose = None
        self.fixed_altitude = 8.0
        self.last_lat, self.last_lon = None, None
        self.uav_lat, self.uav_lon = None, None
        self.prev_tag_source = "GPS"
        self.last_time = time.time()
        self.start_time = self.last_time
        self.latest_tags = {}
        self.in_trajectory_phase = False
        self.trajectory_duration = 0.0
        self.trajectory_hold_duration = 50.0
        self.trajectory_started = False
        self.trajectory_start_time = None
        self.distance_under_threshold_time = None
        self.distance_trigger_threshold = 0.0
        self.wait_before_trajectory = 500.0  # seconds under threshold before starting
        load_trajectory = pd.read_csv('/home/cam_ws/src/dual_pid_docking/dual_pid_docking/Z_trajectory7_4to2in10s_freq100.csv')
        self.vertical_trajectory = load_trajectory.values
        self.trajectory_counter = 0
        self.trajectory_hold_height = 8.0
        self.car_height = 1.76
        self.stale_window = 1.5

        # PID controllers
        self.airspeed_pid = PID(kp=0.08, ki=0.04, kd=0.0, integral_limit=5.0)
        self.altitude_pid = PID(kp=0.052, ki=0.0095, kd=0.25, integral_limit=5.5)
        self.distance_pid = PID(kp=0.5, ki=0.01, kd=0.0, integral_limit=5.0)

        self.heading_log = 0
        self.log_time = []
        self.log_distance_error = []
        self.log_actual_distance = []
        self.log_altitude = []
        self.log_lateral_offset = []
        self.log_tag_height = []  # CV-estimated height from tag (pose.position.x)
        self.last_car_heading = math.radians(255.0)  # default/fallback value

        self.target_lat = None
        self.target_lon = None
        self.last_lat = None
        self.last_lon = None

        self.base_throttle = 0.55
        self.uav_was_ahead = False

        airspeed = self.vehicle.airspeed or 0.0
        airspeed_error = 17.0 - airspeed
        if self.airspeed_pid.ki > 0:
            self.airspeed_pid.integral = (self.base_throttle - self.airspeed_pid.kp * airspeed_error) / self.airspeed_pid.ki

        # Start threads
        self.start_attitude_thread()  # 100 Hz control loop
        #self.start_autotuner_thread()
        self.start_gps_thread()       # 5 Hz GPS sending
        # Start ZN tuner in a separate thread (after control loop is running)
        #threading.Thread(target=self.load_or_run_zn_tuner, daemon=True).start()

    def connect_vehicle(self):
        self.get_logger().info(f"Connecting to vehicle at {self.serial_path}...")
        self.vehicle = connect(self.serial_path)
        self.get_logger().info(f"Connected! Firmware: {self.vehicle.version}")


    def location_callback(self, msg: String):
        try:
            parts = msg.data.strip().split(',')
            lat = float(parts[0].split(':')[1].strip())
            lon = float(parts[1].split(':')[1].strip())

            for part in parts[2:]:
                key_value = part.strip().split(':')
                if len(key_value) == 2 and key_value[0].strip().lower() == 'heading':
                    heading_deg = float(key_value[1].strip())
                    self.last_car_heading = math.radians(heading_deg)

            if self.last_lat is not None:
                self.target_lat = 0.8 * self.last_lat + 0.2 * lat
                self.target_lon = 0.8 * self.last_lon + 0.2 * lon
            else:
                self.target_lat = lat
                self.target_lon = lon

            self.last_lat = self.target_lat
            self.last_lon = self.target_lon
        except Exception as e:
            self.get_logger().error(f"Error parsing location: {e}")

    def apriltag_callback(self, msg):
        if hasattr(msg, 'id'):
            tag_id = msg.id
        else:
            tag_id = 0  # Default if no ID present

        self.latest_tags[tag_id] = (msg.pose, time.time())


    def offset_gps(self, lat, lon, bearing_rad, distance_m):
        """
        Offsets the given lat/lon by distance_m *forward* in the bearing_rad direction.
        Returns new (lat, lon).
        """
        R = 6378137.0  # Earth radius in meters
        lat1 = math.radians(lat)
        lon1 = math.radians(lon)

        d_by_r = distance_m / R

        new_lat = math.asin(
            math.sin(lat1) * math.cos(d_by_r) +
            math.cos(lat1) * math.sin(d_by_r) * math.cos(bearing_rad)
        )

        new_lon = lon1 + math.atan2(
            math.sin(bearing_rad) * math.sin(d_by_r) * math.cos(lat1),
            math.cos(d_by_r) - math.sin(lat1) * math.sin(new_lat)
        )

        return math.degrees(new_lat), math.degrees(new_lon)

    
    def projected_distance_along_heading(self, car_lat, car_lon, car_heading_rad, uav_lat, uav_lon):
        """
        Projects the vector from the car to the UAV onto the car's heading direction.
        Returns positive if the UAV is behind the car, negative if ahead.
        """
        # Approximate flat Earth conversion (good for short distances)
        d_north = geodesic((car_lat, car_lon), (uav_lat, car_lon)).meters
        d_east = geodesic((car_lat, car_lon), (car_lat, uav_lon)).meters
        if uav_lat < car_lat:
            d_north *= -1
        if uav_lon < car_lon:
            d_east *= -1

        heading_x = math.sin(car_heading_rad)
        heading_y = math.cos(car_heading_rad)

        dx = d_east
        dy = d_north

        projection = dx * heading_x + dy * heading_y
        return -projection
    #Right now only used for logging purposes
    def lateral_offset_error(self, car_lat, car_lon, car_heading_rad, uav_lat, uav_lon):
        """
        Compute the perpendicular (left-right) offset from the car's path to the UAV.
        Positive means UAV is to the left of the car's path.
        """
        d_north = geodesic((car_lat, car_lon), (uav_lat, car_lon)).meters
        d_east = geodesic((car_lat, car_lon), (car_lat, uav_lon)).meters
        if uav_lat < car_lat:
            d_north *= -1
        if uav_lon < car_lon:
            d_east *= -1

        # Heading vector
        heading_x = math.sin(car_heading_rad)
        heading_y = math.cos(car_heading_rad)

        # Orthogonal (leftward) vector
        leftward_x = -heading_y
        leftward_y = heading_x

        # Vector from car to UAV
        dx = d_east
        dy = d_north

        # Project UAV vector onto leftward direction
        lateral_error = dx * leftward_x + dy * leftward_y
        return lateral_error
    
    def wrap_angle_deg(self, angle):
        return ((angle + 180) % 360) - 180
    
    def wrap_angle_rad(self, angle):
        """Wrap angle to [-π, π]"""
        return ((angle + math.pi) % (2 * math.pi)) - math.pi 

    def start_gps_thread(self):
        def send_goto_loop():
            start_time = time.time()
            rate = 0.2  # 5 Hz
            while rclpy.ok():
                if self.vehicle.mode.name == "GUIDED" and self.last_lat is not None:
                    elapsed = time.time() - start_time
                    if elapsed >= 2.0:
                        spoof_lat, spoof_lon = self.offset_gps(self.last_lat, self.last_lon, self.last_car_heading, 50.0)
                        self.vehicle.simple_goto(LocationGlobalRelative(spoof_lat, spoof_lon, self.fixed_altitude))
                    else:
                        self.vehicle.simple_goto(LocationGlobalRelative(self.last_lat, self.last_lon, self.fixed_altitude))
                time.sleep(rate)

        thread = threading.Thread(target=send_goto_loop)
        thread.daemon = True
        thread.start()


    def attitude_loop(self):
        log_counter = 0
        while rclpy.ok():
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            elapsed = now - self.start_time

            if self.target_lat is None or self.target_lon is None:
                time.sleep(0.05)
                continue

            if self.vehicle.mode.name != "GUIDED":
                time.sleep(0.05)
                continue
                                
            self.uav_lat = self.vehicle.location.global_relative_frame.lat
            self.uav_lon = self.vehicle.location.global_relative_frame.lon
            altitude = self.vehicle.location.global_relative_frame.alt or 0.0
            airspeed = self.vehicle.airspeed or 0.0
            now = time.time()
            
            use_pose = None
            tag_source = "GPS"
            valid_time_window = 1.0  # seconds

            # --- Collect tag data (pose, timestamp) ---
            tag_data = [(id, *self.latest_tags.get(id, (None, 0.0))) for id in [4, 3, 2, 1, 0]]

            # --- Try valid/fresh detections first ---
            for tag_id, pose, t in tag_data:
                if now - t < valid_time_window:
                    use_pose = pose
                    tag_source = f"AprilTag ID {tag_id}"
                    break

            # --- If no fresh tag, try stale (within stale_window) ---
            if use_pose is None:
                # sort by freshest timestamp first
                tag_data_sorted = sorted(tag_data, key=lambda x: x[2], reverse=True)
                for tag_id, pose, t in tag_data_sorted:
                    if now - t < self.stale_window:
                        use_pose = pose
                        tag_source = f"Stale AprilTag ID {tag_id}"
                        break
            
            in_trajectory_phase = False
            in_hold_phase = False
            desired_distance = 0
            desired_height = None
            if use_pose and not self.trajectory_started:
                if use_pose.pose.position.z < self.distance_trigger_threshold:
                    if self.distance_under_threshold_time is None:
                        self.distance_under_threshold_time = now
                    elif now - self.distance_under_threshold_time > self.wait_before_trajectory:
                        self.trajectory_started = True
                        self.trajectory_start_time = now
                        self.get_logger().info("Trajectory Triggered")
                else:
                    self.distance_under_threshold_time = None
            if self.trajectory_started:
                time_since_traj = now - self.trajectory_start_time
                in_trajectory_phase = time_since_traj < self.trajectory_duration
                in_hold_phase = self.trajectory_duration < time_since_traj < (self.trajectory_duration + self.trajectory_hold_duration)

            if (in_trajectory_phase or in_hold_phase) and use_pose is None:
                self.get_logger().warn("⚠️ AprilTag lost during trajectory/hold phase — reverting to GPS mode.")
                self.trajectory_started = False
                self.trajectory_start_time = None
                self.distance_under_threshold_time = None
                in_trajectory_phase = False
                in_hold_phase = False
            
            if in_trajectory_phase:
                desired_distance = 0.0
                self.altitude_pid.kd = 0.05
                self.altitude_pid.kp = 0.1
                if self.trajectory_counter<len(self.vertical_trajectory):
                
                    desired_height = (self.vertical_trajectory[self.trajectory_counter]).item()
                    self.trajectory_counter += 1
                else:
                    desired_height = self.trajectory_hold_height
            
            if in_hold_phase:
                self.stale_window = 10.0
                self.altitude_pid.kp = 0.01
                desired_height = 1.0
            else:
                self.stale_window = 1.5

            if use_pose is not None:
                positive_height = -1*use_pose.pose.position.x
                self.distance_pid.kp = 0.2  # Lower gain when tag is seen
                spoofed_forward_distance = 500 # Not accepted anymore, just here to make the l1 command work
                altitude = self.car_height + positive_height
                if self.prev_tag_source != tag_source:
                    self.distance_pid.integral = 0.0
                    self.distance_pid.last_error = 0.0
                lateral_error = use_pose.pose.position.y
                self.send_custom_l1_external_nav(float(lateral_error), float(spoofed_forward_distance), 1)
                if desired_height is None:
                    altitude_error = self.fixed_altitude - altitude
                else:
                    altitude_error = desired_height - positive_height
                distance_error = use_pose.pose.position.z - desired_distance
                dist_to_target = use_pose.pose.position.z
            else:
                tag_source = "GPS"
                altitude = self.vehicle.location.global_relative_frame.alt or 0.0
                if self.last_lat is not None and self.last_lon is not None:
                    dist_to_target = geodesic((self.uav_lat, self.uav_lon), (self.last_lat, self.last_lon)).meters

                distance_error = dist_to_target - desired_distance
                lateral_error = self.lateral_offset_error(self.last_lat, self.last_lon, self.last_car_heading, self.uav_lat, self.uav_lon)
                altitude_error = self.fixed_altitude - (self.vehicle.location.global_relative_frame.alt or 0.0)
            self.prev_dist = dist_to_target

            # --- Target airspeed with PID ---
            target_airspeed = 16.0 + self.distance_pid.update(distance_error, dt)  # baseline matching speed

            # --- Final clamp ---
            target_airspeed = max(10.0, min(target_airspeed, 22.0))

            # --- Altitude + Pitch ---
            pitch_cmd = self.altitude_pid.update(altitude_error, dt)
            pitch_cmd = max(min(pitch_cmd, math.radians(15)), math.radians(-5))

            # --- Throttle for airspeed ---
            airspeed_error = target_airspeed - airspeed
            throttle_adjust = self.airspeed_pid.update(airspeed_error, dt)
            throttle_cmd = self.base_throttle + throttle_adjust
            throttle_cmd = max(0.0, min(throttle_cmd, 1.0))

            #Just for logging
            if math.isfinite(altitude):
                elapsed = now - self.start_time
                self.log_time.append(elapsed)
                self.log_distance_error.append(distance_error)
                self.log_actual_distance.append(dist_to_target)
                self.log_altitude.append(altitude)
                if in_trajectory_phase or in_hold_phase:
                    self.log_lateral_offset.append(lateral_error)
                else:
                    self.log_lateral_offset.append(None)
            
            tag_height_str = f"{positive_height:.2f}m" if use_pose is not None else "N/A"
            tag_lateral_str = f"{use_pose.pose.position.y:.2f}m" if use_pose is not None else "N/A"

            #--- Attitude Command ---#
            q = euler_to_quaternion(0.0, pitch_cmd, 0.0)
            self.vehicle._master.mav.set_attitude_target_send(
                int((now - self.start_time) * 1000),
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                0b00000100,
                q,
                0.0, 0.0, 0.0,
                throttle_cmd
            )

            if log_counter % 10 == 0:
                if in_trajectory_phase:
                    flight_phase = "DESCENT"
                elif in_hold_phase:
                    flight_phase = "HOLD"
                else:
                    flight_phase = "NORMAL"

                self.get_logger().info(
                    f"[{tag_source}] Phase: {flight_phase} | Dist: {dist_to_target:.2f}m | TgtAS: {target_airspeed:.2f} | AS: {airspeed:.2f} | "
                    f"Alt: {altitude:.2f} | Throttle: {throttle_cmd:.2f} | Pitch(deg): {math.degrees(pitch_cmd):.2f} | "
                    f"Lateral Offset: {lateral_error:.2f}m | Altitude error: {altitude_error:.2f}m |"
                    f"TagHeight: {tag_height_str} | Heading Car: {math.degrees(self.last_car_heading):.2f}° | TagLateral: {tag_lateral_str} | DemandedHeight: {desired_height} |"
                )
            log_counter += 1
            
            time.sleep(0.01)


    def send_custom_l1_external_nav(self, xtrack_error, wp_distance, enable):
        self.vehicle._master.mav.l1_external_nav_send(
            xtrack_error,     # param1: cross-track error (left/right)
            wp_distance,      # param2: distance to WP
            enable            # param3: 1=enable, 0=disable
        )

    def start_attitude_thread(self):
        thread = threading.Thread(target=self.attitude_loop)
        thread.daemon = True
        thread.start()

def main(args=None):
    rclpy.init(args=args)
    gps_follower = GpsFollower()
    try:
        rclpy.spin(gps_follower)
    except KeyboardInterrupt:
        pass
    finally:
        gps_follower.destroy_node()

        csv_filename = "lateral_offset_data.csv"
        with open(csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Lateral Offset (m)", "car heading"])
            for t, offset in zip(gps_follower.log_time, gps_follower.log_lateral_offset):
                writer.writerow([t, offset])
        print(f"✅ CSV saved to: {os.path.abspath(csv_filename)}")

if __name__ == '__main__':
    main()
