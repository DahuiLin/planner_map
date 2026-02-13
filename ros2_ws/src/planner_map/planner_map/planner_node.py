#!/usr/bin/env python3
"""
Planner Node - Lanelet2-based path planning node for road navigation
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import json
from .lanelet2_map_loader import Lanelet2MapLoader


class PlannerNode(Node):
    """
    ROS2 node for path planning and navigation using Lanelet2
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
        # Subscribe to vehicle GPS position (NavSatFix)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        # State variables
        self.current_map = None
        self.current_goal = None
        self.current_path = None
        self.vehicle_gps = None  # Vehicle GPS position from /fix topic

        # Lanelet2 loader for routing
        self.map_loader = Lanelet2MapLoader()
        self.map_loaded = False

        # Load OSM file if specified
        osm_file = self.get_parameter('osm_file').value
        if osm_file:
            self.load_osm_file(osm_file)

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Planner node initialized with Lanelet2 support')

    def load_osm_file(self, osm_file: str):
        """Load OSM file for routing using Lanelet2"""
        try:
            import os
            if os.path.exists(osm_file):
                self.get_logger().info(f'Loading OSM file with Lanelet2: {osm_file}')
                num_lanelets, num_areas = self.map_loader.load_osm_file(osm_file)
                self.map_loaded = True
                self.get_logger().info(f'Lanelet2 map loaded: {num_lanelets} lanelets, {num_areas} areas')
            else:
                self.get_logger().warn(f'OSM file not found: {osm_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load OSM file with Lanelet2: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def map_metadata_callback(self, msg):
        """Handle map metadata updates"""
        try:
            metadata = json.loads(msg.data)
            if metadata.get('type') == 'lanelet2':
                self.get_logger().info('Received Lanelet2 map metadata')
        except Exception as e:
            self.get_logger().error(f'Failed to parse map metadata: {e}')

    def gps_callback(self, msg):
        """Handle vehicle GPS position updates from /fix topic"""
        self.vehicle_gps = msg
        self.get_logger().debug(
            f'Vehicle GPS updated: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, '
            f'status={msg.status.status}'
        )

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
        """Plan path using Lanelet2 routing"""
        if self.current_goal is None:
            self.get_logger().warn('No goal set, cannot plan path')
            return

        if not self.map_loaded:
            self.get_logger().warn('Lanelet2 map not loaded, cannot plan path')
            self._create_simple_path()
            return

        # Get start position from vehicle GPS
        # The vehicle GPS position (/fix topic) is always the starting point
        if self.vehicle_gps is None:
            self.get_logger().warn('No vehicle GPS position available (/fix topic), cannot plan path')
            return

        # Start position from GPS
        start_lat = self.vehicle_gps.latitude
        start_lon = self.vehicle_gps.longitude

        # Get goal position - convert from XY to lat/lon
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        goal_lat, goal_lon = self.map_loader.xy_to_latlon(goal_x, goal_y)

        self.get_logger().info(
            f'Planning path with Lanelet2 from GPS ({start_lat:.6f}, {start_lon:.6f}) to '
            f'({goal_lat:.6f}, {goal_lon:.6f})'
        )

        # Find shortest path using Lanelet2 routing
        try:
            path_latlon = self.map_loader.find_shortest_path(
                start_lat, start_lon,
                goal_lat, goal_lon
            )

            if path_latlon:
                # Convert to ROS Path message
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                path_msg.header.stamp = self.get_clock().now().to_msg()

                for lat, lon in path_latlon:
                    # Convert lat/lon to XY for ROS path
                    x, y = self.map_loader.latlon_to_xy(lat, lon)

                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0

                    path_msg.poses.append(pose)

                self.current_path = path_msg
                self.path_pub.publish(path_msg)
                self.get_logger().info(f'Published Lanelet2 path with {len(path_latlon)} waypoints')
            else:
                self.get_logger().warn('No path found using Lanelet2 routing')
                self._create_simple_path()

        except Exception as e:
            self.get_logger().error(f'Lanelet2 path planning failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
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
