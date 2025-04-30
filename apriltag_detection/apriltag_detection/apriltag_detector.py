import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation
from apriltag_interfaces.msg import TagPoseStamped  # Your custom message


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

        # Static transform: camera -> base_link
        self.t_cam_in_base = np.array([0, -0.2, 0])
        self.R_cam_in_base = Rotation.from_euler('x', -25, degrees=True).as_matrix()

        self.get_logger().info('🧠 Manual AprilTag Transformer running — with tag ID included.')

    def tf_callback(self, msg):
        for transform in msg.transforms:
            tag_id = None

            # Replace these with your actual frame names
            if transform.child_frame_id == 'big':
                tag_id = 0
            elif transform.child_frame_id == 'small':
                tag_id = 1
            else:
                continue  # Skip unrelated transforms

            # Position in camera frame
            t = transform.transform.translation
            p_camera = np.array([t.x, t.y, t.z])
            p_base = self.R_cam_in_base @ p_camera + self.t_cam_in_base

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
