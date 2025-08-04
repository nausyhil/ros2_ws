import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np

class MapToLaserScan(Node):
    def __init__(self):
        super().__init__('map_to_laserscan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('scan_rate', 1.0)
        self.declare_parameter('range_max', 5.0)

        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.timer = self.create_timer(1.0 / self.scan_rate, self.publish_scan)

        self.map = None

    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info('Map received.')

    def publish_scan(self):
        if self.map is None:
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = -np.pi / 2
        scan.angle_max = np.pi / 2
        scan.angle_increment = np.pi / 180  # 1 degree
        scan.range_min = 0.0
        scan.range_max = self.range_max

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [self.range_max] * num_readings  # Fake constant range

        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = MapToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()