#!/usr/bin/env python3
"""
Map Server Node - Manages and publishes OSM map data
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose
import numpy as np
import json
import os
from .osm_map_loader import OSMMapLoader


class MapServer(Node):
    """
    ROS2 node for serving OSM map data
    """

    def __init__(self):
        super().__init__('map_server')

        # Parameters
        self.declare_parameter('osm_file', '')
        self.declare_parameter('publish_rate', 1.0)

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.map_metadata_pub = self.create_publisher(String, '/map_metadata', 10)

        # OSM Map Loader
        self.osm_loader = OSMMapLoader()
        self.map_loaded = False

        # Load OSM file
        osm_file = self.get_parameter('osm_file').value
        if osm_file and os.path.exists(osm_file):
            self.load_osm_map(osm_file)
        else:
            self.get_logger().warn(f'OSM file not found or not specified: {osm_file}')
            self.get_logger().info('Creating default sample map...')
            self.initialize_default_map()

        # Timer to publish map periodically
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)

        self.get_logger().info('Map server initialized')

    def load_osm_map(self, osm_file_path: str):
        """Load OSM map from file"""
        try:
            self.get_logger().info(f'Loading OSM map from: {osm_file_path}')
            num_nodes, num_ways = self.osm_loader.load_osm_file(osm_file_path)
            self.get_logger().info(f'Loaded {num_nodes} nodes and {num_ways} ways')

            # Convert OSM data to occupancy grid for ROS2
            self.create_occupancy_grid_from_osm()
            self.map_loaded = True

            # Publish metadata
            self.publish_map_metadata()

        except Exception as e:
            self.get_logger().error(f'Failed to load OSM map: {e}')
            self.initialize_default_map()

    def create_occupancy_grid_from_osm(self):
        """Create an occupancy grid from OSM road network"""
        self.map_msg = OccupancyGrid()

        # Map metadata
        if not hasattr(self, '_map_resolution'):
            # Declare map_resolution parameter with default 1.0 (meters per cell)
            self._map_resolution = float(self.declare_parameter('map_resolution', 1.0).value)
        self.map_msg.info.resolution = self._map_resolution

        # Get bounds from OSM data
        bounds = self.osm_loader.bounds
        if bounds:
            # Calculate map dimensions in local coordinates
            min_lat, max_lat = bounds['min_lat'], bounds['max_lat']
            min_lon, max_lon = bounds['min_lon'], bounds['max_lon']

            # Convert corners to XY
            min_x, min_y = self.osm_loader.latlon_to_xy(min_lat, min_lon)
            max_x, max_y = self.osm_loader.latlon_to_xy(max_lat, max_lon)

            # Add padding
            padding = 50  # meters
            min_x -= padding
            min_y -= padding
            max_x += padding
            max_y += padding

            # Calculate dimensions
            width = int((max_x - min_x) / self.map_msg.info.resolution)
            height = int((max_y - min_y) / self.map_msg.info.resolution)

            self.map_msg.info.width = width
            self.map_msg.info.height = height
            self.map_msg.info.origin.position.x = min_x
            self.map_msg.info.origin.position.y = min_y
            self.map_msg.info.origin.position.z = 0.0
            self.map_msg.info.origin.orientation.w = 1.0

            # Create grid (unknown space by default)
            grid = np.full((height, width), -1, dtype=np.int8)

            # Mark roads as free space (0)
            ways = self.osm_loader.get_ways_as_linestrings()
            for way in ways:
                coords = way['coordinates']
                for i in range(len(coords) - 1):
                    lon1, lat1 = coords[i]
                    lon2, lat2 = coords[i + 1]

                    x1, y1 = self.osm_loader.latlon_to_xy(lat1, lon1)
                    x2, y2 = self.osm_loader.latlon_to_xy(lat2, lon2)

                    # Rasterize line
                    self._draw_line(grid, x1, y1, x2, y2, min_x, min_y)

            self.map_msg.data = grid.flatten().tolist()

            self.get_logger().info(f'Created occupancy grid: {width}x{height}')
        else:
            self.initialize_default_map()

    def _draw_line(self, grid, x1, y1, x2, y2, origin_x, origin_y):
        """Draw a line on the grid using Bresenham's algorithm"""
        resolution = self.map_msg.info.resolution
        road_width = 3  # cells

        # Convert world coordinates to grid coordinates
        gx1 = int((x1 - origin_x) / resolution)
        gy1 = int((y1 - origin_y) / resolution)
        gx2 = int((x2 - origin_x) / resolution)
        gy2 = int((y2 - origin_y) / resolution)

        # Bresenham's line algorithm
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        err = dx - dy

        while True:
            # Draw with road width
            for dy_offset in range(-road_width // 2, road_width // 2 + 1):
                for dx_offset in range(-road_width // 2, road_width // 2 + 1):
                    gy = gy1 + dy_offset
                    gx = gx1 + dx_offset
                    if 0 <= gy < grid.shape[0] and 0 <= gx < grid.shape[1]:
                        grid[gy, gx] = 0  # Free space

            if gx1 == gx2 and gy1 == gy2:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx1 += sx
            if e2 < dx:
                err += dx
                gy1 += sy

    def initialize_default_map(self):
        """Initialize a simple default map"""
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

        self.get_logger().info(f'Initialized default map: {self.map_msg.info.width}x{self.map_msg.info.height}')

    def publish_map(self):
        """Publish the map periodically"""
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_msg.header.frame_id = 'map'
        self.map_pub.publish(self.map_msg)

    def publish_map_metadata(self):
        """Publish map metadata including OSM-specific information"""
        if not self.map_loaded:
            return

        metadata = {
            'type': 'osm',
            'bounds': self.osm_loader.bounds,
            'num_nodes': len(self.osm_loader.nodes),
            'num_ways': len(self.osm_loader.ways),
            'ways': self.osm_loader.get_ways_as_linestrings()[:100]  # Limit to first 100 for size
        }

        msg = String()
        msg.data = json.dumps(metadata)
        self.map_metadata_pub.publish(msg)

        self.get_logger().info('Published map metadata')


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
