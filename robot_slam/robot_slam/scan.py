import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

class LaserToPointConverter(Node):
    def __init__(self):
        super().__init__('laser_to_point_converter_node')
        qos_profile = QoSProfile(depth=30)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile)
        self.marker_pub = self.create_publisher(MarkerArray, 'laser_markers', 30)
        self.seq = 0

    def laser_callback(self, msg):
        markers = self.convert_to_markers(msg)
        self.marker_pub.publish(markers)

    def convert_to_markers(self, msg):
        markers = MarkerArray()
        for i, r in enumerate(msg.ranges):
            if not math.isnan(r):
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                marker = Marker()
                marker.header.frame_id = "base_link"  # Set the header for each marker
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "laser_scan"
                marker.id = i
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.color.r = 1.0
                marker.color.a = 1.0

                p = Point()
                p.x = x
                p.y = y
                p.z = 0.0
                marker.points.append(p)

                markers.markers.append(marker)

        return markers


def main(args=None):
    rclpy.init(args=args)
    converter_node = LaserToPointConverter()
    rclpy.spin(converter_node)
    converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
