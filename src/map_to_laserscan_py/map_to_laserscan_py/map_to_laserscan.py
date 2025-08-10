import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformException
import math

class MapToLaserScan(Node):
    def __init__(self):
        super().__init__('map_to_laserscan')
        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', 'base_link')  # Changed to base_link
        self.declare_parameter('scan_rate', 10.0)  # Increased for better responsiveness
        self.declare_parameter('range_max', 5.0)
        self.declare_parameter('range_min', 0.1)

        # Get parameters
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers and subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.timer = self.create_timer(1.0 / self.scan_rate, self.publish_scan)

        self.map = None
        self.get_logger().info('Map to LaserScan node initialized')

    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info('Map received')

    def get_robot_pose_in_map(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.frame_id,
                rclpy.time.Time())
            return transform
        except TransformException as ex:
            self.get_logger().warning(f'Could not get transform: {ex}')
            return None

    def world_to_map(self, wx, wy):
        """Convert world coordinates to map cell coordinates"""
        if self.map is None:
            return None, None
        
        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        mx = int((wx - origin_x) / resolution)
        my = int((wy - origin_y) / resolution)

        if (mx < 0 or mx >= self.map.info.width or 
            my < 0 or my >= self.map.info.height):
            return None, None

        return mx, my

    def raycast(self, start_x, start_y, angle):
        """Cast a ray from start position and return distance to obstacle"""
        if self.map is None:
            return self.range_max

        resolution = self.map.info.resolution
        step_size = resolution / 2.0  # Half resolution for better accuracy

        # Initialize ray position
        cur_x = start_x
        cur_y = start_y
        distance = 0.0

        # Ray direction components
        dx = math.cos(angle)
        dy = math.sin(angle)

        while distance < self.range_max:
            # Get map cell for current position
            mx, my = self.world_to_map(cur_x, cur_y)
            
            if mx is None or my is None:
                return self.range_max

            # Check if cell is occupied
            index = my * self.map.info.width + mx
            if self.map.data[index] > 50:  # Cell is occupied
                return max(distance, self.range_min)

            # Move ray forward
            cur_x += dx * step_size
            cur_y += dy * step_size
            distance += step_size

        return self.range_max

    def publish_scan(self):
        if self.map is None:
            return

        # Get robot's pose in map frame
        transform = self.get_robot_pose_in_map()
        if transform is None:
            return

        # Create LaserScan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = -np.pi / 2
        scan.angle_max = np.pi / 2
        scan.angle_increment = np.pi / 180  # 1-degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Get robot's position
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        # Calculate orientation from quaternion
        q = transform.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # Calculate ranges
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = []

        for i in range(num_readings):
            # Calculate ray angle in map frame
            angle = scan.angle_min + i * scan.angle_increment + yaw
            # Cast ray and get distance
            range_val = self.raycast(robot_x, robot_y, angle)
            scan.ranges.append(range_val)

        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = MapToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()