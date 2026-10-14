#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class WallFollowerVisualizerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_visualizer_node')
        
        # Subscriptions & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/foxglove_markers', 10)
        
        self.get_logger().info("Wall Follower & Foxglove Visualizer Node Initialized.")

    def scan_callback(self, msg: LaserScan):
        # 1. Parse LaserScan polar to Cartesian coordinates
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment/2, msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Filter invalid ranges
        valid_idx = (ranges >= msg.range_min) & (ranges <= msg.range_max) & ~np.isnan(ranges) & ~np.isinf(ranges)
        r_valid = ranges[valid_idx]
        th_valid = angles[valid_idx]
        
        x = r_valid * np.cos(th_valid)
        y = r_valid * np.sin(th_valid)
        
        # TODO: Implement your custom RANSAC or Split-and-Merge algorithm here to extract line segments
        
        # 2. Publish Visualization Markers for Foxglove Studio
        self.publish_markers(x, y)

    def publish_markers(self, x_pts, y_pts):
        marker_array = MarkerArray()
        
        # Example Marker: Render extracted point cloud or line segments
        points_marker = Marker()
        points_marker.header.frame_id = "laser_frame"
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "lidar_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        
        # Scale for Foxglove point display
        points_marker.scale.x = 0.03
        points_marker.scale.y = 0.03
        
        # Color (Green points)
        points_marker.color.r = 0.0
        points_marker.color.g = 1.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0
        
        for px, py in zip(x_pts, y_pts):
            from geometry_msgs.msg import Point
            p = Point()
            p.x = float(px)
            p.y = float(py)
            p.z = 0.0
            points_marker.points.append(p)
            
        marker_array.markers.append(points_marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
