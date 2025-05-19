import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation
from apriltag_interfaces.msg import TagPoseStamped


class AprilTagPoseManualTransform(Node):
    def __init__(self):
        super().__init__('apriltag_pose_manual_transform')

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        self.publisher = self.create_publisher(TagPoseStamped, '/apriltag/pose_in_base', 10)

        # Static transform from camera to base_link
        self.t_cam_in_base = np.array([0, -0.2, 0])
        self.R_cam_in_base = Rotation.from_euler('x', -35, degrees=True).as_matrix()

        # Per-tag positional offsets (in meters)
        self.tag_offsets = {
            0: np.array([0.0, 0.0, 0.0]),
            1: np.array([0.0, 0.0, 0.56]),
            2: np.array([0.0, 0.0, 0.30]),
            3: np.array([0.165, 0.0, -0.23]),
            4: np.array([0.0, 0.0, -0.24]),
        }

        self.get_logger().info('🧠 Manual AprilTag Transformer running — with per-tag offsets.')

    def tf_callback(self, msg):
        for transform in msg.transforms:
            tag_id = None

            # Match known tag frame names to IDs
            if transform.child_frame_id == 'zero':
                tag_id = 0
            elif transform.child_frame_id == 'one':
                tag_id = 1
            elif transform.child_frame_id == 'two':
                tag_id = 2
            elif transform.child_frame_id == 'three':
                tag_id = 3
            elif transform.child_frame_id == 'four':
                tag_id = 4
            else:
                continue

            # Position in camera frame
            t = transform.transform.translation
            p_camera = np.array([t.x, t.y, t.z])

            # Rotate into base_link frame
            p_base = self.R_cam_in_base @ p_camera + self.t_cam_in_base

            # Apply per-tag positional offset
            if tag_id in self.tag_offsets:
                p_base += self.tag_offsets[tag_id]

            # Orientation
            q = transform.transform.rotation
            q_camera = np.array([q.x, q.y, q.z, q.w])
            r_camera = Rotation.from_quat(q_camera)
            r_base = Rotation.from_matrix(self.R_cam_in_base @ r_camera.as_matrix())
            q_base = r_base.as_quat()

            # Fill PoseStamped
            pose = PoseStamped()
            pose.header.stamp = transform.header.stamp
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = p_base[0]
            pose.pose.position.y = p_base[1]
            pose.pose.position.z = p_base[2]
            pose.pose.orientation.x = q_base[0]
            pose.pose.orientation.y = q_base[1]
            pose.pose.orientation.z = q_base[2]
            pose.pose.orientation.w = q_base[3]

            # Fill and publish custom message
            tag_msg = TagPoseStamped()
            tag_msg.id = tag_id
            tag_msg.pose = pose
            self.publisher.publish(tag_msg)

            self.get_logger().info(
                f'📍 Tag {tag_id} in base_link: x={p_base[0]:.2f}, y={p_base[1]:.2f}, z={p_base[2]:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseManualTransform()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
