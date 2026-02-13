#!/usr/bin/env python3
"""
ROS2 Bridge - Connects ROS2 nodes with FastAPI web interface
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
import requests
import json
from threading import Thread
import time


class ROS2WebBridge(Node):
    """
    Bridge node that forwards data between ROS2 and the web interface
    """
    
    def __init__(self):
        super().__init__('ros2_web_bridge')
        
        # Web interface URL
        self.web_api_url = "http://web:8000/api"
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Timer to check for web goals
        self.timer = self.create_timer(1.0, self.check_web_goals)
        
        self.last_goal = None
        
        self.get_logger().info('ROS2-Web bridge initialized')
        self.get_logger().info(f'Web API URL: {self.web_api_url}')
    
    def map_callback(self, msg):
        """Forward map updates to web interface"""
        try:
            # Convert map to JSON-serializable format
            map_data = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'data': list(msg.data)
            }
            
            # Send to web API
            response = requests.post(
                f"{self.web_api_url}/map",
                json=map_data,
                timeout=1.0
            )
            
            if response.status_code == 200:
                self.get_logger().debug('Map data sent to web interface')
            
        except Exception as e:
            self.get_logger().warn(f'Failed to send map to web: {e}')
    
    def path_callback(self, msg):
        """Forward path updates to web interface"""
        try:
            # Convert path to JSON-serializable format
            path_data = []
            for pose in msg.poses:
                path_data.append({
                    'position': {
                        'x': pose.pose.position.x,
                        'y': pose.pose.position.y,
                        'z': pose.pose.position.z
                    },
                    'orientation': {
                        'x': pose.pose.orientation.x,
                        'y': pose.pose.orientation.y,
                        'z': pose.pose.orientation.z,
                        'w': pose.pose.orientation.w
                    }
                })
            
            # Note: You might want to add a path endpoint in the web API
            self.get_logger().debug(f'Path received with {len(path_data)} poses')
            
        except Exception as e:
            self.get_logger().warn(f'Failed to process path: {e}')
    
    def check_web_goals(self):
        """Check web interface for new goals and publish to ROS2"""
        # This is a simplified implementation
        # In production, you'd want to use WebSocket or a proper messaging system
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ROS2WebBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
