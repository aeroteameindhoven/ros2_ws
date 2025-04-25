import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation

class AprilTagPoseManualTransform(Node):
    def __init__(self):
        super().__init__('apriltag_pose_manual_transform')

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        self.publisher = self.create_publisher(PoseStamped, '/apriltag/pose_in_base', 10)

        # Static transform: camera -> base_link
        self.t_cam_in_base = np.array([0, -0.2, 0])
        self.R_cam_in_base = Rotation.from_euler('x', -25, degrees=True).as_matrix()

        self.get_logger().info('🧠 Manual AprilTag Transformer running — no TF used.')

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id != 'tag36h11:0':
                continue

            # Tag position in camera frame
            t = transform.transform.translation
            p_camera = np.array([t.x, t.y, t.z])

            # Transform into base_link frame
            p_base = self.R_cam_in_base @ p_camera + self.t_cam_in_base

            # Orientation stays the same for now (we’ll rotate it manually too if needed)
            q = transform.transform.rotation
            q_camera = np.array([q.x, q.y, q.z, q.w])

            # Rotate orientation to base_link frame
            r_camera = Rotation.from_quat(q_camera)
            r_base = Rotation.from_matrix(self.R_cam_in_base @ r_camera.as_matrix())
            q_base = r_base.as_quat()  # [x, y, z, w]

            # Create new PoseStamped
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

            self.publisher.publish(pose)

            self.get_logger().info(
                f'📍 Tag in base_link: x={p_base[0]:.2f}, y={p_base[1]:.2f}, z={p_base[2]:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseManualTransform()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
