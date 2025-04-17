import rclpy
from rclpy.node import Node

class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        self.get_logger().info("Node 1 is running")

def main(args=None):
    rclpy.init(args=args)
    node = Node1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
