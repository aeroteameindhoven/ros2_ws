import rclpy
from rclpy.node import Node
from dronekit import connect, Vehicle
from std_msgs.msg import String

class UAV1Node(Node):
    def __init__(self):
        super().__init__('uav1_node')
        self.publisher = self.create_publisher(String, 'uav1/location', 10)
        self.serial_path = '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30I5PMQ-if00-port0'
        self.vehicle: Vehicle = None
        self.connect_vehicle()

    def connect_vehicle(self):
        try:
            self.get_logger().info(f"[UAV1] Connecting to {self.serial_path}...")
            self.vehicle = connect(self.serial_path, wait_ready=True, baud=57600, timeout=100)
            self.get_logger().info(f"[UAV1] Connected with firmware: {self.vehicle.version}")
            self.create_timer(1/5, self.publish_location)
        except Exception as e:
            self.get_logger().error(f"[UAV1] Connection error: {e}")

    def publish_location(self):
        if not self.vehicle:
            return

        loc = self.vehicle.location.global_frame
        if loc.lat is not None and loc.lon is not None and loc.alt is not None:
            msg = String()
            msg.data = f"Lat: {loc.lat:.7f}, Lon: {loc.lon:.7f}, Alt: {loc.alt:.2f}"
            self.publisher.publish(msg)
        else:
            self.get_logger().warn("[UAV1] GPS location not available yet.")

def main(args=None):
    rclpy.init(args=args)
    node = UAV1Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
