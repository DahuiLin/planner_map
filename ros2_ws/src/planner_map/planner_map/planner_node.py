#!/usr/bin/env python3
"""
Planner Node - OSM-based path planning node for road navigation
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
import json
from .osm_map_loader import OSMMapLoader


class PlannerNode(Node):
    """
    ROS2 node for path planning and navigation using OSM road networks
    """

    def __init__(self):
        super().__init__('planner_node')

        # Parameters
        self.declare_parameter('osm_file', '')

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
        self.map_metadata_sub = self.create_subscription(
            String,
            '/map_metadata',
            self.map_metadata_callback,
            10
        )

        # State variables
        self.current_map = None
        self.current_goal = None
        self.current_path = None
        self.start_pose = None  # Could be set by localization

        # OSM loader for routing
        self.osm_loader = OSMMapLoader()
        self.osm_loaded = False

        # Load OSM file if specified
        osm_file = self.get_parameter('osm_file').value
        if osm_file:
            self.load_osm_file(osm_file)

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Planner node initialized with OSM support')

    def load_osm_file(self, osm_file: str):
        """Load OSM file for routing"""
        try:
            import os
            if os.path.exists(osm_file):
                self.get_logger().info(f'Loading OSM file for routing: {osm_file}')
                num_nodes, num_ways = self.osm_loader.load_osm_file(osm_file)
                self.osm_loaded = True
                self.get_logger().info(f'OSM loaded: {num_nodes} nodes, {num_ways} ways')
            else:
                self.get_logger().warn(f'OSM file not found: {osm_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load OSM file: {e}')

    def map_metadata_callback(self, msg):
        """Handle map metadata updates"""
        try:
            metadata = json.loads(msg.data)
            if metadata.get('type') == 'osm':
                self.get_logger().info('Received OSM map metadata')
        except Exception as e:
            self.get_logger().error(f'Failed to parse map metadata: {e}')

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
        """Plan path using OSM road network"""
        if self.current_goal is None:
            return

        if not self.osm_loaded:
            self.get_logger().warn('OSM not loaded, cannot plan path')
            self._create_simple_path()
            return

        # Get start position (for now, use origin as start)
        # In real scenario, this would come from localization
        if self.start_pose is None:
            start_x, start_y = 0.0, 0.0
        else:
            start_x = self.start_pose.pose.position.x
            start_y = self.start_pose.pose.position.y

        # Get goal position
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y

        # Convert XY to lat/lon
        start_lat, start_lon = self.osm_loader.xy_to_latlon(start_x, start_y)
        goal_lat, goal_lon = self.osm_loader.xy_to_latlon(goal_x, goal_y)

        self.get_logger().info(
            f'Planning path from ({start_lat:.6f}, {start_lon:.6f}) to '
            f'({goal_lat:.6f}, {goal_lon:.6f})'
        )

        # Find shortest path on road network
        try:
            path_latlon = self.osm_loader.find_shortest_path(
                start_lat, start_lon,
                goal_lat, goal_lon
            )

            if path_latlon:
                # Convert to ROS Path message
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                path_msg.header.stamp = self.get_clock().now().to_msg()

                for lat, lon in path_latlon:
                    # Convert back to XY
                    x, y = self.osm_loader.latlon_to_xy(lat, lon)

                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0

                    path_msg.poses.append(pose)

                self.current_path = path_msg
                self.path_pub.publish(path_msg)
                self.get_logger().info(f'Published path with {len(path_latlon)} waypoints')
            else:
                self.get_logger().warn('No path found on road network')
                self._create_simple_path()

        except Exception as e:
            self.get_logger().error(f'Path planning failed: {e}')
            self._create_simple_path()

    def _create_simple_path(self):
        """Create a simple straight-line path as fallback"""
        if self.current_goal is None:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Just add the goal
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose = self.current_goal.pose
        path_msg.poses.append(pose)

        self.current_path = path_msg
        self.path_pub.publish(path_msg)
        self.get_logger().info('Published simple path')

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
