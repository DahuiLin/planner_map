#!/usr/bin/env python3
"""
Map Server Node - Manages and publishes map data
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np


class MapServer(Node):
    """
    ROS2 node for serving map data
    """
    
    def __init__(self):
        super().__init__('map_server')
        
        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer to publish map periodically
        self.timer = self.create_timer(1.0, self.publish_map)
        
        # Initialize a simple test map
        self.initialize_map()
        
        self.get_logger().info('Map server initialized')
    
    def initialize_map(self):
        """Initialize a simple test map"""
        self.map_msg = OccupancyGrid()
        
        # Map metadata
        self.map_msg.info.resolution = 0.05  # 5cm per cell
        self.map_msg.info.width = 100
        self.map_msg.info.height = 100
        self.map_msg.info.origin.position.x = -2.5
        self.map_msg.info.origin.position.y = -2.5
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        
        # Create a simple map (all free space for demonstration)
        data = np.zeros((self.map_msg.info.height, self.map_msg.info.width), dtype=np.int8)
        self.map_msg.data = data.flatten().tolist()
        
        self.get_logger().info(f'Initialized map: {self.map_msg.info.width}x{self.map_msg.info.height}')
    
    def publish_map(self):
        """Publish the map periodically"""
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.header.frame_id = 'map'
        self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
