#!/usr/bin/env python3
"""
OSM Map Loader - Loads and processes OpenStreetMap files using osmium
"""
import osmium
import json
from typing import Dict, List, Tuple
import networkx as nx
from pyproj import Transformer
import numpy as np


class OSMMapHandler(osmium.SimpleHandler):
    """Handler for processing OSM data"""

    def __init__(self):
        super().__init__()
        self.nodes = {}  # node_id -> (lat, lon)
        self.ways = []   # list of ways with nodes and tags
        self.road_types = {
            'motorway', 'trunk', 'primary', 'secondary', 'tertiary',
            'residential', 'service', 'motorway_link', 'trunk_link',
            'primary_link', 'secondary_link', 'tertiary_link',
            'unclassified', 'road', 'living_street'
        }

    def node(self, n):
        """Process OSM nodes"""
        self.nodes[n.id] = (n.location.lat, n.location.lon)

    def way(self, w):
        """Process OSM ways (roads)"""
        # Check if this way is a road
        tags = {tag.k: tag.v for tag in w.tags}

        if 'highway' in tags and tags['highway'] in self.road_types:
            node_ids = [n.ref for n in w.nodes]
            self.ways.append({
                'id': w.id,
                'nodes': node_ids,
                'tags': tags
            })


class OSMMapLoader:
    """
    Loads OpenStreetMap files and converts them to a routable graph
    """

    def __init__(self):
        self.graph = nx.DiGraph()
        self.nodes = {}
        self.ways = []
        self.transformer = None
        self.bounds = None

    def load_osm_file(self, osm_file_path: str):
        """
        Load OSM file and build road network graph

        Args:
            osm_file_path: Path to .osm file
        """
        handler = OSMMapHandler()
        handler.apply_file(osm_file_path)

        self.nodes = handler.nodes
        self.ways = handler.ways

        # Calculate bounds
        if self.nodes:
            lats = [lat for lat, lon in self.nodes.values()]
            lons = [lon for lat, lon in self.nodes.values()]
            self.bounds = {
                'min_lat': min(lats),
                'max_lat': max(lats),
                'min_lon': min(lons),
                'max_lon': max(lons)
            }

            # Create transformer for lat/lon to local coordinates
            center_lat = (self.bounds['min_lat'] + self.bounds['max_lat']) / 2
            center_lon = (self.bounds['min_lon'] + self.bounds['max_lon']) / 2

            # Use UTM projection centered at map center
            self.transformer = Transformer.from_crs(
                "EPSG:4326",  # WGS84
                f"+proj=tmerc +lat_0={center_lat} +lon_0={center_lon} +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs",
                always_xy=True
            )

        self._build_graph()

        return len(self.nodes), len(self.ways)

    def _build_graph(self):
        """Build NetworkX graph from OSM data"""
        # Add all nodes to graph
        for node_id, (lat, lon) in self.nodes.items():
            x, y = self.transformer.transform(lon, lat)
            self.graph.add_node(node_id, lat=lat, lon=lon, x=x, y=y)

        # Add edges from ways
        for way in self.ways:
            nodes = way['nodes']
            tags = way['tags']

            # Determine if way is one-way
            oneway = tags.get('oneway', 'no') == 'yes'

            # Get max speed (default based on road type)
            max_speed = self._get_max_speed(tags)

            # Add edges
            for i in range(len(nodes) - 1):
                node1, node2 = nodes[i], nodes[i + 1]

                if node1 in self.graph and node2 in self.graph:
                    # Calculate distance
                    x1, y1 = self.graph.nodes[node1]['x'], self.graph.nodes[node1]['y']
                    x2, y2 = self.graph.nodes[node2]['x'], self.graph.nodes[node2]['y']
                    distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                    # Add edge with weight as distance
                    self.graph.add_edge(node1, node2,
                                       distance=distance,
                                       max_speed=max_speed,
                                       way_id=way['id'],
                                       highway=tags.get('highway', 'road'))

                    # Add reverse edge if not oneway
                    if not oneway:
                        self.graph.add_edge(node2, node1,
                                           distance=distance,
                                           max_speed=max_speed,
                                           way_id=way['id'],
                                           highway=tags.get('highway', 'road'))

    def _get_max_speed(self, tags: dict) -> float:
        """Get max speed for a way, default based on road type"""
        if 'maxspeed' in tags:
            try:
                return float(tags['maxspeed'])
            except:
                pass

        # Default speeds by road type (km/h)
        highway = tags.get('highway', 'road')
        speed_defaults = {
            'motorway': 120,
            'trunk': 100,
            'primary': 80,
            'secondary': 60,
            'tertiary': 50,
            'residential': 30,
            'service': 20,
            'living_street': 10
        }

        return speed_defaults.get(highway, 50)

    def find_shortest_path(self, start_lat: float, start_lon: float,
                          end_lat: float, end_lon: float) -> List[Tuple[float, float]]:
        """
        Find shortest path between two lat/lon coordinates

        Args:
            start_lat, start_lon: Starting coordinates
            end_lat, end_lon: Ending coordinates

        Returns:
            List of (lat, lon) tuples representing the path
        """
        # Find nearest nodes to start and end
        start_node = self._find_nearest_node(start_lat, start_lon)
        end_node = self._find_nearest_node(end_lat, end_lon)

        if start_node is None or end_node is None:
            return []

        try:
            # Use Dijkstra's algorithm with distance as weight
            path_nodes = nx.shortest_path(self.graph, start_node, end_node, weight='distance')

            # Convert node IDs to lat/lon coordinates
            path = []
            for node_id in path_nodes:
                node = self.graph.nodes[node_id]
                path.append((node['lat'], node['lon']))

            return path
        except nx.NetworkXNoPath:
            return []

    def _find_nearest_node(self, lat: float, lon: float) -> int:
        """Find nearest node to given coordinates"""
        min_dist = float('inf')
        nearest_node = None

        for node_id, node_data in self.graph.nodes(data=True):
            dist = ((node_data['lat'] - lat)**2 + (node_data['lon'] - lon)**2)**0.5
            if dist < min_dist:
                min_dist = dist
                nearest_node = node_id

        return nearest_node

    def get_nodes_in_bounds(self) -> List[Dict]:
        """Get all road network nodes with their coordinates"""
        nodes_list = []
        for node_id, data in self.graph.nodes(data=True):
            nodes_list.append({
                'id': node_id,
                'lat': data['lat'],
                'lon': data['lon'],
                'x': data['x'],
                'y': data['y']
            })
        return nodes_list

    def get_ways_as_linestrings(self) -> List[Dict]:
        """Get all ways as linestrings for visualization"""
        linestrings = []
        for way in self.ways:
            coords = []
            for node_id in way['nodes']:
                if node_id in self.graph:
                    node = self.graph.nodes[node_id]
                    coords.append([node['lon'], node['lat']])

            if len(coords) >= 2:
                linestrings.append({
                    'id': way['id'],
                    'coordinates': coords,
                    'highway': way['tags'].get('highway', 'road'),
                    'name': way['tags'].get('name', '')
                })

        return linestrings

    def latlon_to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert lat/lon to local XY coordinates"""
        if self.transformer:
            x, y = self.transformer.transform(lon, lat)
            return x, y
        return 0.0, 0.0

    def xy_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """Convert local XY coordinates to lat/lon"""
        if self.transformer:
            lon, lat = self.transformer.transform(x, y, direction='INVERSE')
            return lat, lon
        return 0.0, 0.0
