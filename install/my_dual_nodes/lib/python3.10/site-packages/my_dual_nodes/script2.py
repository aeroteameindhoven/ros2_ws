import rclpy
from rclpy.node import Node

class Node2(Node):
    def __init__(self):
        super().__init__('node2')
        self.get_logger().info("Node 2 is running")

def main(args=None):
    rclpy.init(args=args)
    node = Node2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
