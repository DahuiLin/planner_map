# OSM Map Support Documentation

## Overview

This repository now supports OpenStreetMap (OSM) files for road-based navigation. The system uses OSM road networks to plan the shortest routes for car navigation.

## Features

- **OSM File Loading**: Load .osm files containing road networks
- **Road Network Routing**: Find shortest paths along roads using Dijkstra's algorithm
- **Web Interface Integration**: Select start and end points on the web interface
- **ROS2 Integration**: Full integration with ROS2 navigation stack

## Quick Start

### 1. Prepare an OSM File

You can obtain OSM files in several ways:

#### Option A: Use the Sample Map
A sample OSM map is provided in `config/sample_map.osm` for testing.

#### Option B: Download from OpenStreetMap
1. Go to [openstreetmap.org](https://www.openstreetmap.org/)
2. Navigate to your area of interest
3. Click "Export" at the top
4. Select "Manually select a different area" if needed
5. Click "Export" button to download the .osm file

#### Option C: Use Overpass API
For programmatic access:
```bash
curl -o my_map.osm "http://overpass-api.de/api/map?bbox=left,bottom,right,top"
```

### 2. Launch with OSM Support

```bash
# Using Docker Compose
docker-compose up --build

# The system will look for OSM file in config directory
# You can specify a custom OSM file by modifying docker-compose.yml
```

### 3. Using Custom OSM Files

Edit `docker-compose.yml` to specify your OSM file:

```yaml
services:
  ros2:
    command: >
      bash -c "source /opt/ros/humble/setup.bash &&
               source /tmp/lanelet2_ws/install/setup.bash &&
               source /workspace/ros2_ws/install/setup.bash &&
               ros2 launch planner_map planner_map.launch.py osm_file:=/workspace/config/your_map.osm"
```

Or when running manually:
```bash
ros2 launch planner_map planner_map.launch.py osm_file:=/path/to/your/map.osm
```

## Architecture

### Components

1. **osm_map_loader.py**: Core OSM loading and routing library
   - Loads .osm files using osmium library
   - Builds a NetworkX graph of the road network
   - Implements shortest path finding using Dijkstra's algorithm
   - Converts between lat/lon and local XY coordinates

2. **map_server.py**: ROS2 node for map management
   - Loads OSM files and publishes as OccupancyGrid
   - Publishes road network metadata
   - Converts OSM data to ROS2 messages

3. **planner_node.py**: ROS2 node for path planning
   - Receives goal poses from web interface
   - Plans shortest routes on road network
   - Publishes paths as ROS2 Path messages

### Data Flow

```
OSM File → osm_map_loader → map_server → /map topic
                          ↓
                    planner_node
                          ↓
                  find_shortest_path
                          ↓
                   /planned_path topic
                          ↓
                    ros2_web_bridge
                          ↓
                    Web Interface
```

## Web Interface

The web interface has been updated to work with OSM maps:

### Setting a Goal
1. Open http://localhost:8000
2. Click on the map to set a goal point
3. The system will find the nearest road node
4. A route will be calculated along the road network
5. The route is displayed on the map

### Features
- Interactive map with road network overlay
- Click-to-set goal functionality
- Real-time route visualization
- Route distance and waypoint information

## API Endpoints

### New Endpoints for OSM

- `GET /api/map/osm`: Get OSM-derived road network data
  - Returns road network metadata in the internal JSON format (not GeoJSON)
  - Structure matches data posted to `/api/map/metadata` (includes custom `ways` list, node positions, and way geometries)

- `POST /api/route`: Request route planning
  ```json
  {
    "start": {"lat": 40.748, "lon": -73.987},
    "end": {"lat": 40.750, "lon": -73.985}
  }
  ```
  Returns: Path as array of lat/lon coordinates

## Road Types Supported

The system recognizes these OSM highway types:
- motorway
- trunk
- primary
- secondary
- tertiary
- residential
- service
- unclassified
- living_street

And their corresponding `_link` variants.

## Configuration

### Map Server Parameters

Edit `ros2_ws/src/planner_map/config/params.yaml`:

```yaml
map_server:
  ros__parameters:
    osm_file: "/path/to/map.osm"
    publish_rate: 1.0
    map_resolution: 1.0  # meters per cell for occupancy grid
```

### Planner Parameters

```yaml
planner_node:
  ros__parameters:
    osm_file: "/path/to/map.osm"
    planning_algorithm: "dijkstra"
```

## Dependencies

### Python Libraries
- `osmium>=3.6.0`: OSM file parsing
- `networkx>=3.1`: Graph algorithms
- `pyproj>=3.6.0`: Coordinate transformations
- `numpy`: Numerical operations

### System Libraries
- `libboost-all-dev`: Boost C++ libraries
- `libeigen3-dev`: Linear algebra library
- `libgeographic-dev`: Geographic computations
- `libpugixml-dev`: XML parsing

### Optional: Lanelet2
For advanced lanelet map support (not required for basic OSM):
- Lanelet2 library (installed from source in Docker)

## Troubleshooting

### OSM File Not Loading

**Problem**: "OSM file not found" error

**Solution**:
- Check the file path is correct
- Ensure the file is mounted in Docker: add to `docker-compose.yml`
  ```yaml
  volumes:
    - ./your_maps:/workspace/maps
  ```
- Use absolute paths in launch files

### No Route Found

**Problem**: "No path found on road network"

**Possible causes**:
1. Start or end point is too far from roads
   - System finds nearest node within the network
   - If map is small, points may be outside

2. Disconnected road network
   - OSM data may have separate, unconnected road segments
   - Use a more complete map export

3. One-way restrictions
   - Some roads may be one-way
   - Check OSM tags: `oneway=yes`

**Solution**: Use a larger, more complete OSM export

### Memory Issues with Large Maps

**Problem**: Out of memory when loading large OSM files

**Solution**:
1. Use a smaller geographic area
2. Increase Docker memory limit:
   ```yaml
   services:
     ros2:
       mem_limit: 4g
   ```
3. Filter roads by type (edit `osm_map_loader.py`)

### Coordinate Transformation Errors

**Problem**: Points appear in wrong locations

**Solution**:
- Ensure OSM file has valid lat/lon coordinates
- Check coordinate reference system (CRS)
- Verify map bounds are reasonable

## Advanced Usage

### Custom Road Filtering

Edit `osm_map_loader.py` to filter roads:

```python
self.road_types = {
    'motorway', 'trunk', 'primary',  # Only major roads
}
```

### Speed Limits

The router considers max speed from OSM tags:
- Uses `maxspeed` tag if available
- Falls back to defaults by road type
- Affects route optimization (can weight by time instead of distance)

### Multi-Modal Routing

To add bicycle or pedestrian routing, modify:
```python
# In osm_map_loader.py
self.transport_types = ['highway', 'cycleway', 'footway']
```

## Examples

### Example 1: Basic Usage
```bash
# Launch with sample map
ros2 launch planner_map planner_map.launch.py \
  osm_file:=/workspace/config/sample_map.osm
```

### Example 2: Custom Map
```bash
# Download NYC area
curl -o nyc.osm "http://overpass-api.de/api/map?bbox=-74.0,40.7,-73.9,40.8"

# Launch with custom map
ros2 launch planner_map planner_map.launch.py \
  osm_file:=/path/to/nyc.osm
```

### Example 3: Programmatic Route Request
```python
import requests

response = requests.post('http://localhost:8000/api/route', json={
    'start': {'lat': 40.748, 'lon': -73.987},
    'end': {'lat': 40.750, 'lon': -73.985}
})

route = response.json()['path']
print(f"Route has {len(route)} waypoints")
```

## Performance

### Map Loading Times
- Small maps (< 100 ways): < 1 second
- Medium maps (< 10,000 ways): 1-10 seconds
- Large maps (> 100,000 ways): 10-60 seconds

### Route Planning Times
- Typical urban routes: < 100ms
- Long-distance routes: 100-500ms
- Very large networks: 500ms - 2s

## Integration with Lanelet2

While this implementation uses OSM directly, it can be extended to support Lanelet2 format:

1. Lanelet2 provides HD map format for autonomous driving
2. Includes lane-level detail and traffic rules
3. Can be converted from/to OSM

To use Lanelet2:
```python
# Future implementation
from lanelet2.io import load
from lanelet2.routing import RoutingGraph

map_data = load("map.osm", lanelet2.projection.UtmProjector(...))
traffic_rules = create_traffic_rules(...)
routing_graph = RoutingGraph(map_data, traffic_rules)
```

## Contributing

To contribute OSM/routing features:
1. Test with various OSM files
2. Add support for more road types
3. Implement time-based routing
4. Add traffic rule awareness
5. Improve coordinate system handling

## References

- [OpenStreetMap](https://www.openstreetmap.org/)
- [OSM Wiki - Map Features](https://wiki.openstreetmap.org/wiki/Map_features)
- [Overpass API](https://overpass-api.de/)
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [NetworkX Documentation](https://networkx.org/)
- [osmium Documentation](https://osmcode.org/pyosmium/)
