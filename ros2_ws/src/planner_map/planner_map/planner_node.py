#!/usr/bin/env python3
"""
Planner Node - Main path planning node
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
import json


class PlannerNode(Node):
    """
    ROS2 node for path planning and navigation
    """
    
    def __init__(self):
        super().__init__('planner_node')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # State variables
        self.current_map = None
        self.current_goal = None
        self.current_path = None
        
        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Planner node initialized')
    
    def goal_callback(self, msg):
        """Handle new goal pose"""
        self.current_goal = msg
        self.get_logger().info(f'Received goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.plan_path()
    
    def map_callback(self, msg):
        """Handle map updates"""
        self.current_map = msg
        self.get_logger().info('Map updated')
    
    def plan_path(self):
        """Simple path planning algorithm"""
        if self.current_goal is None:
            return
        
        # Create a simple path (placeholder for actual planning algorithm)
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # For demonstration, create a straight line path
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose = self.current_goal.pose
        path_msg.poses.append(pose)
        
        self.current_path = path_msg
        self.path_pub.publish(path_msg)
        self.get_logger().info('Published planned path')
    
    def timer_callback(self):
        """Periodic update callback"""
        # Placeholder for periodic control updates
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
