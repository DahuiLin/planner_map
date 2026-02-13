#!/usr/bin/env python3
"""
ROS2 Bridge - Connects ROS2 nodes with FastAPI web interface
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
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

        self.map_metadata_sub = self.create_subscription(
            String,
            '/map_metadata',
            self.map_metadata_callback,
            10
        )

        self.spline_trajectory_sub = self.create_subscription(
            Path,
            '/spline_trajectory',
            self.spline_trajectory_callback,
            10
        )

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.spline_trigger_pub = self.create_publisher(String, '/calculate_spline_trajectory', 10)

        # Timer to check for web goals
        self.timer = self.create_timer(1.0, self.check_web_goals)
        self.timer_spline = self.create_timer(1.0, self.check_spline_trigger)

        self.last_goal = None
        self.last_spline_trigger = None

        self.get_logger().info('ROS2-Web bridge initialized with OSM support')
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
                timeout=5.0  # Longer timeout to prevent frequent failures
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

            # Send path to web API
            response = requests.post(
                f"{self.web_api_url}/path",
                json={'path': path_data},
                timeout=5.0
            )

            if response.status_code == 200:
                self.get_logger().debug(f'Path sent to web interface ({len(path_data)} poses)')

        except Exception as e:
            self.get_logger().warn(f'Failed to send path to web: {e}')

    def map_metadata_callback(self, msg):
        """Forward OSM map metadata to web interface"""
        try:
            # Parse metadata JSON
            metadata = json.loads(msg.data)

            # Send to web API
            response = requests.post(
                f"{self.web_api_url}/map/metadata",
                json=metadata,
                timeout=5.0
            )

            if response.status_code == 200:
                self.get_logger().info('OSM map metadata sent to web interface')

        except Exception as e:
            self.get_logger().warn(f'Failed to send map metadata to web: {e}')

    def spline_trajectory_callback(self, msg):
        """Forward spline trajectory updates to web interface"""
        try:
            # Convert trajectory to JSON-serializable format
            trajectory_data = []
            for pose in msg.poses:
                trajectory_data.append({
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

            # Send trajectory to web API
            response = requests.post(
                f"{self.web_api_url}/trajectory",
                json={'trajectory': trajectory_data},
                timeout=5.0
            )

            if response.status_code == 200:
                self.get_logger().info(f'Spline trajectory sent to web interface ({len(trajectory_data)} points)')

        except Exception as e:
            self.get_logger().warn(f'Failed to send spline trajectory to web: {e}')

    def check_web_goals(self):
        """Check web interface for new goals and publish to ROS2"""
        try:
            # Poll the web API for new goals
            response = requests.get(
                f"{self.web_api_url}/goal/latest",
                timeout=2.0
            )
            
            if response.status_code == 200:
                data = response.json()
                
                # Check if there's a new goal
                if data.get('goal') is not None:
                    goal_data = data['goal']
                    
                    # Check if it's a new goal (different from last one)
                    goal_str = json.dumps(goal_data)
                    if goal_str != self.last_goal:
                        self.last_goal = goal_str
                        
                        # Create ROS2 PoseStamped message
                        goal_msg = PoseStamped()
                        goal_msg.header.frame_id = 'map'
                        goal_msg.header.stamp = self.get_clock().now().to_msg()
                        
                        # Set position
                        pose_data = goal_data['pose']
                        goal_msg.pose.position.x = pose_data['position']['x']
                        goal_msg.pose.position.y = pose_data['position']['y']
                        goal_msg.pose.position.z = pose_data['position']['z']
                        
                        # Set orientation
                        goal_msg.pose.orientation.x = pose_data['orientation']['x']
                        goal_msg.pose.orientation.y = pose_data['orientation']['y']
                        goal_msg.pose.orientation.z = pose_data['orientation']['z']
                        goal_msg.pose.orientation.w = pose_data['orientation']['w']
                        
                        # Publish to ROS2
                        self.goal_pub.publish(goal_msg)
                        self.get_logger().info(
                            f'Published new goal from web: '
                            f'x={goal_msg.pose.position.x:.2f}, '
                            f'y={goal_msg.pose.position.y:.2f}'
                        )
                        
        except requests.exceptions.RequestException as e:
            # Don't log every timeout, only every 10th to avoid spam
            if not hasattr(self, '_error_count'):
                self._error_count = 0
            self._error_count += 1
            if self._error_count % 10 == 0:
                self.get_logger().debug(f'Could not reach web API: {e}')
        except Exception as e:
            self.get_logger().warn(f'Error checking web goals: {e}')

    def check_spline_trigger(self):
        """Check web interface for spline calculation trigger and publish to ROS2"""
        try:
            # Poll the web API for spline calculation trigger
            response = requests.get(
                f"{self.web_api_url}/trajectory/trigger",
                timeout=2.0
            )

            if response.status_code == 200:
                data = response.json()

                # Check if there's a new trigger
                if data.get('trigger') is not None and data['trigger']:
                    trigger_str = data.get('timestamp', '')

                    # Check if it's a new trigger (different from last one)
                    if trigger_str != self.last_spline_trigger:
                        self.last_spline_trigger = trigger_str

                        # Publish trigger to ROS2
                        trigger_msg = String()
                        trigger_msg.data = 'calculate'
                        self.spline_trigger_pub.publish(trigger_msg)

                        self.get_logger().info('Published spline calculation trigger from web')

        except requests.exceptions.RequestException:
            # Don't log timeouts to avoid spam
            pass
        except Exception as e:
            self.get_logger().warn(f'Error checking spline trigger: {e}')


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
