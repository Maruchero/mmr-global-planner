#!/usr/bin/env python3

import csv
import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class CenterlinePublisher(Node):
    def __init__(self):
        super().__init__('dummy_marker_publisher')

        # Create publisher for Marker message
        self.publisher_ = self.create_publisher(
            Marker, '/planning/center_line_completed', 10)

        centerline = self.load_csv_data('/mmr/src/mmrGlobalPlanner/global_planner/input/varano_x_y_r_v.csv')

        # Set up a timer to publish messages
        # timer_period = 1.0  # seconds
        # self.timer = self.create_timer(timer_period, self.publish_centerline)
        self.get_logger().info('Centerline Publisher Node Started')

        self.publish_centerline(centerline)
    
    def load_csv_data(self, csv_file):
        """Load marker data from CSV file with columns: x, y, r, v"""
        data = []
        try:
            if not os.path.exists(csv_file):
                self.get_logger().error(f"CSV file not found: {csv_file}")
                return None
            
            with open(csv_file, 'r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    try:
                        data.append({
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'r': float(row.get('r', 0.1)),  # default radius
                            'v': float(row.get('v', 0.0))   # default velocity
                        })
                    except (ValueError, KeyError) as e:
                        self.get_logger().warn(f"Skipping invalid row: {row}. Error: {e}")
            return data
        except Exception as e:
            self.get_logger().error(f"Error loading CSV: {e}")
            return None

    def publish_centerline(self, centerline):
        # Create a Marker message
        marker_msg = Marker()
        
        # Add all points from CSV
        marker_msg.points = []
        for point in centerline:
            marker_msg.points.append(Point(
                x=point['x'],
                y=point['y'],
                z=0.0  # Assuming 2D data
            ))

        # Add some optional metadata (if your Marker message has these fields)
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = 'map'
        marker_msg.id = 1
        marker_msg.type = Marker.POINTS

        # Publish the message
        self.publisher_.publish(marker_msg)
        self.get_logger().info('Publishing centerline data')


def main(args=None):
    rclpy.init(args=args)
    node = CenterlinePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
