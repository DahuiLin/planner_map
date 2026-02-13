#!/usr/bin/env python3
"""
Lanelet2 Map Loader - Loads and processes OpenStreetMap files using Lanelet2
"""
import os
from typing import Dict, List, Tuple, Optional
import numpy as np

try:
    import lanelet2
    from lanelet2.core import getId, AttributeMap, Point3d, LineString3d
    from lanelet2.io import Origin, load
    from lanelet2.projection import UtmProjector
    from lanelet2.traffic_rules import create as createTrafficRules
    from lanelet2.routing import RoutingGraph, RoutingCostDistance
    LANELET2_AVAILABLE = True
except ImportError:
    LANELET2_AVAILABLE = False
    print("Warning: Lanelet2 not available. Install lanelet2_python to use this loader.")


class Lanelet2MapLoader:
    """
    Loads OpenStreetMap files using Lanelet2 library and provides routing capabilities
    """

    def __init__(self):
        if not LANELET2_AVAILABLE:
            raise ImportError("Lanelet2 library is not available. Please install lanelet2_python.")

        self.map = None
        self.routing_graph = None
        self.traffic_rules = None
        self.projector = None
        self.origin = None
        self.bounds = None

    def load_osm_file(self, osm_file_path: str, origin_lat: float = None, origin_lon: float = None):
        """
        Load OSM file using Lanelet2

        Args:
            osm_file_path: Path to .osm file
            origin_lat: Origin latitude for projection (optional, auto-detected from map)
            origin_lon: Origin longitude for projection (optional, auto-detected from map)

        Returns:
            Tuple of (num_lanelets, num_areas)
        """
        if not os.path.exists(osm_file_path):
            raise FileNotFoundError(f"OSM file not found: {osm_file_path}")

        # Create projector
        # If origin not specified, Lanelet2 will auto-detect from the map
        if origin_lat is not None and origin_lon is not None:
            self.origin = Origin(origin_lat, origin_lon)
            self.projector = UtmProjector(self.origin)
        else:
            # Use UtmProjector without explicit origin (will be determined from map bounds)
            self.projector = UtmProjector(Origin(0, 0))

        # Load the map
        self.map = load(osm_file_path, self.projector)

        # Calculate bounds
        self._calculate_bounds()

        # Create traffic rules for vehicles (car)
        # This defines which lanelets are drivable
        self.traffic_rules = createTrafficRules("de", "vehicle")  # German traffic rules for vehicles

        # Create routing graph
        # This builds the graph used for route planning
        self.routing_graph = RoutingGraph(self.map, self.traffic_rules, [RoutingCostDistance()])

        num_lanelets = len(self.map.laneletLayer)
        num_areas = len(self.map.areaLayer)

        return num_lanelets, num_areas

    def _calculate_bounds(self):
        """Calculate bounding box of the map"""
        if self.map is None:
            return

        # Get all points from lanelets
        all_points = []
        for lanelet in self.map.laneletLayer:
            for point in lanelet.leftBound:
                all_points.append((point.x, point.y, point.z))
            for point in lanelet.rightBound:
                all_points.append((point.x, point.y, point.z))

        if all_points:
            points_array = np.array(all_points)
            self.bounds = {
                'min_x': float(np.min(points_array[:, 0])),
                'max_x': float(np.max(points_array[:, 0])),
                'min_y': float(np.min(points_array[:, 1])),
                'max_y': float(np.max(points_array[:, 1])),
            }

    def find_shortest_path(self, start_lat: float, start_lon: float,
                          end_lat: float, end_lon: float) -> List[Tuple[float, float]]:
        """
        Find shortest path between two GPS coordinates using Lanelet2 routing

        Args:
            start_lat, start_lon: Starting GPS coordinates
            end_lat, end_lon: Ending GPS coordinates

        Returns:
            List of (lat, lon) tuples representing the path
        """
        if self.routing_graph is None:
            return []

        # Convert GPS to local coordinates
        start_x, start_y = self.latlon_to_xy(start_lat, start_lon)
        end_x, end_y = self.latlon_to_xy(end_lat, end_lon)

        # Find nearest lanelets to start and end positions
        start_point = Point3d(getId(), start_x, start_y, 0.0)
        end_point = Point3d(getId(), end_x, end_y, 0.0)

        start_lanelets = self._find_nearest_lanelets(start_point, max_distance=50.0)
        end_lanelets = self._find_nearest_lanelets(end_point, max_distance=50.0)

        if not start_lanelets or not end_lanelets:
            return []

        # Try to find route between the nearest lanelets
        route = None
        for start_ll in start_lanelets:
            for end_ll in end_lanelets:
                try:
                    optional_route = self.routing_graph.getRoute(start_ll, end_ll)
                    if optional_route:
                        route = optional_route
                        break
                except:
                    continue
            if route:
                break

        if not route:
            return []

        # Extract path from route
        path = []
        shortest_path = route.shortestPath()

        for lanelet in shortest_path:
            # Get centerline points
            centerline = lanelet.centerline
            for point in centerline:
                lat, lon = self.xy_to_latlon(point.x, point.y)
                path.append((lat, lon))

        return path

    def _find_nearest_lanelets(self, point: 'Point3d', max_distance: float = 50.0) -> List:
        """
        Find nearest lanelets to a given point

        Args:
            point: Point3d to search from
            max_distance: Maximum search distance in meters

        Returns:
            List of nearest lanelets
        """
        if self.map is None:
            return []

        nearest = []
        min_dist = max_distance

        for lanelet in self.map.laneletLayer:
            # Calculate distance to lanelet centerline
            centerline = lanelet.centerline
            for center_point in centerline:
                dist = np.sqrt(
                    (point.x - center_point.x)**2 +
                    (point.y - center_point.y)**2
                )
                if dist < min_dist:
                    if dist < max_distance:
                        nearest.append(lanelet)
                    min_dist = dist
                    break  # Only check first point for performance

        return nearest[:5]  # Return up to 5 nearest lanelets

    def latlon_to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Convert GPS coordinates to local XY coordinates

        Args:
            lat: Latitude
            lon: Longitude

        Returns:
            Tuple of (x, y) in meters
        """
        if self.projector is None:
            return 0.0, 0.0

        point = self.projector.forward(lanelet2.core.GPSPoint(lat, lon))
        return point.x, point.y

    def xy_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert local XY coordinates to GPS coordinates

        Args:
            x: X coordinate in meters
            y: Y coordinate in meters

        Returns:
            Tuple of (lat, lon)
        """
        if self.projector is None:
            return 0.0, 0.0

        point = lanelet2.core.BasicPoint3d(x, y, 0.0)
        gps_point = self.projector.reverse(point)
        return gps_point.lat, gps_point.lon

    def get_lanelets_as_linestrings(self) -> List[Dict]:
        """
        Get all lanelets as linestrings for visualization

        Returns:
            List of dictionaries containing lanelet information
        """
        if self.map is None:
            return []

        linestrings = []
        for lanelet in self.map.laneletLayer:
            # Get centerline coordinates
            coords = []
            for point in lanelet.centerline:
                lat, lon = self.xy_to_latlon(point.x, point.y)
                coords.append([lon, lat])  # GeoJSON format: [lon, lat]

            if len(coords) >= 2:
                # Get attributes
                attributes = {}
                for key in lanelet.attributes.keys():
                    attributes[key] = lanelet.attributes[key]

                linestrings.append({
                    'id': lanelet.id,
                    'coordinates': coords,
                    'type': attributes.get('type', 'lanelet'),
                    'subtype': attributes.get('subtype', ''),
                    'speed_limit': attributes.get('speed_limit', ''),
                })

        return linestrings

    def get_map_info(self) -> Dict:
        """
        Get general information about the loaded map

        Returns:
            Dictionary with map information
        """
        if self.map is None:
            return {}

        return {
            'num_lanelets': len(self.map.laneletLayer),
            'num_areas': len(self.map.areaLayer),
            'num_regulatory_elements': len(self.map.regulatoryElementLayer),
            'bounds': self.bounds,
        }
