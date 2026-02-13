#!/usr/bin/env python3
"""
Map Server Node - Manages and publishes Lanelet2 map data
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose
import numpy as np
import json
import os
from .lanelet2_map_loader import Lanelet2MapLoader


class MapServer(Node):
    """
    ROS2 node for serving Lanelet2 map data
    """

    def __init__(self):
        super().__init__('map_server')

        # Parameters
        self.declare_parameter('osm_file', '')
        self.declare_parameter('publish_rate', 1.0)

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.map_metadata_pub = self.create_publisher(String, '/map_metadata', 10)

        # Lanelet2 Map Loader
        self.map_loader = Lanelet2MapLoader()
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

        self.get_logger().info('Map server initialized with Lanelet2')

    def load_osm_map(self, osm_file_path: str):
        """Load OSM map from file using Lanelet2"""
        try:
            self.get_logger().info(f'Loading Lanelet2 map from: {osm_file_path}')
            num_lanelets, num_areas = self.map_loader.load_osm_file(osm_file_path)
            self.get_logger().info(f'Loaded {num_lanelets} lanelets and {num_areas} areas')

            # Convert Lanelet2 data to occupancy grid for ROS2
            self.create_occupancy_grid_from_lanelet2()
            self.map_loaded = True

            # Publish metadata
            self.publish_map_metadata()

        except Exception as e:
            self.get_logger().error(f'Failed to load Lanelet2 map: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.initialize_default_map()

    def create_occupancy_grid_from_lanelet2(self):
        """Create an occupancy grid from Lanelet2 map"""
        self.map_msg = OccupancyGrid()

        # Map metadata
        if not hasattr(self, '_map_resolution'):
            # Declare map_resolution parameter with default 1.0 (meters per cell)
            self._map_resolution = float(self.declare_parameter('map_resolution', 1.0).value)
        self.map_msg.info.resolution = self._map_resolution

        # Get bounds from Lanelet2 data
        bounds = self.map_loader.bounds
        if bounds:
            # Get bounds in XY coordinates
            min_x = bounds['min_x']
            max_x = bounds['max_x']
            min_y = bounds['min_y']
            max_y = bounds['max_y']

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

            # Mark lanelets as free space (0)
            lanelets = self.map_loader.get_lanelets_as_linestrings()
            for lanelet in lanelets:
                coords = lanelet['coordinates']
                for i in range(len(coords) - 1):
                    lon1, lat1 = coords[i]
                    lon2, lat2 = coords[i + 1]

                    x1, y1 = self.map_loader.latlon_to_xy(lat1, lon1)
                    x2, y2 = self.map_loader.latlon_to_xy(lat2, lon2)

                    # Rasterize line
                    self._draw_line(grid, x1, y1, x2, y2, min_x, min_y)

            self.map_msg.data = grid.flatten().tolist()

            self.get_logger().info(f'Created occupancy grid from Lanelet2: {width}x{height}')
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
        """Publish map metadata including Lanelet2-specific information"""
        if not self.map_loaded:
            return

        map_info = self.map_loader.get_map_info()
        lanelets = self.map_loader.get_lanelets_as_linestrings()[:100]  # Limit to first 100 for size

        metadata = {
            'type': 'lanelet2',
            'bounds': map_info.get('bounds'),
            'num_lanelets': map_info.get('num_lanelets', 0),
            'num_areas': map_info.get('num_areas', 0),
            'num_regulatory_elements': map_info.get('num_regulatory_elements', 0),
            'lanelets': lanelets
        }

        msg = String()
        msg.data = json.dumps(metadata)
        self.map_metadata_pub.publish(msg)

        self.get_logger().info('Published Lanelet2 map metadata')


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
