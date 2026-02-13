# Lanelet2 Integration Guide

## Overview

This project has been migrated from using OSM + NetworkX to **Lanelet2** for map processing and route planning. Lanelet2 provides a more sophisticated framework for lane-level mapping and routing specifically designed for autonomous vehicles.

## What Changed

### Previous Implementation (OSM + NetworkX)
- Used `osmium` for parsing OSM files
- Used `networkx` for graph construction and Dijkstra's algorithm
- Used `pyproj` for coordinate transformations
- Basic road-level routing without lane information

### New Implementation (Lanelet2)
- **Lanelet2 reads OSM files directly** using its native parser
- Lane-level map representation with traffic rules
- Built-in routing graph with traffic rule awareness
- Better support for regulatory elements (traffic lights, speed limits, etc.)
- Native coordinate projection handling

## Key Components

### 1. Lanelet2MapLoader (`lanelet2_map_loader.py`)

This is the new map loader that uses Lanelet2's built-in OSM parser.

**Main Features:**
- **Reads OSM files directly** using Lanelet2's `lanelet2.io.load()` function
- Creates routing graph with traffic rules (German vehicle rules by default)
- Provides coordinate conversion (GPS ↔ local XY)
- Finds shortest paths using Lanelet2's routing algorithms
- Extracts lanelet information for visualization

**Key Methods:**
- `load_osm_file(osm_file_path)`: Load and parse OSM file
- `find_shortest_path(start_lat, start_lon, end_lat, end_lon)`: Find route between GPS coordinates
- `latlon_to_xy(lat, lon)`: Convert GPS to local coordinates
- `xy_to_latlon(x, y)`: Convert local coordinates to GPS
- `get_lanelets_as_linestrings()`: Get lanelet data for visualization

### 2. Vehicle GPS Position (`/fix` topic)

**NEW REQUIREMENT:** The system now requires a vehicle GPS position from a ROS2 topic.

- **Topic Name:** `/fix`
- **Message Type:** `sensor_msgs/NavSatFix`
- **Purpose:** Provides the vehicle's current GPS position, which is always used as the starting point for route planning

**Important:** Route planning will not work without an active GPS signal on the `/fix` topic.

### 3. Route Planning Flow

1. **Vehicle Position (Start):** Read from `/fix` topic (GPS coordinates)
2. **Goal Position (End):** Selected by user in web interface (converted from XY to GPS)
3. **Lanelet2 Routing:**
   - Find nearest lanelets to start and end positions
   - Use Lanelet2's routing graph to find shortest path
   - Extract centerline waypoints from the route
4. **Path Publication:**
   - Convert waypoints to ROS2 `nav_msgs/Path` message
   - Publish to `/planned_path` topic for RViz visualization
   - Forward to web interface via bridge for browser visualization

## Dependencies

### ROS2 Packages
- `rclpy`: ROS2 Python client library
- `sensor_msgs`: For NavSatFix messages
- `nav_msgs`: For Path messages
- `geometry_msgs`: For pose messages

### Lanelet2
Lanelet2 is built from source in the Docker container. The Dockerfile includes:

```dockerfile
# Install lanelet2 from source
RUN mkdir -p /tmp/lanelet2_ws/src && cd /tmp/lanelet2_ws/src && \
    git clone https://github.com/fzi-forschungszentrum-informatik/lanelet2.git && \
    cd /tmp/lanelet2_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Python Dependencies (requirements.txt)
- `requests`: For web API communication
- `numpy`: For numerical operations

**Removed dependencies:**
- `networkx`: No longer needed (Lanelet2 handles routing)
- `osmium`: No longer needed (Lanelet2 parses OSM)
- `pyproj`: No longer needed (Lanelet2 handles projections)

## Usage

### 1. Ensure GPS Data is Available

Before route planning can work, you need to publish GPS data to the `/fix` topic:

```bash
# Example: Publish a test GPS position
ros2 topic pub /fix sensor_msgs/NavSatFix "{
  latitude: 48.98403,
  longitude: 8.39014,
  altitude: 115.0,
  status: {status: 0, service: 1}
}" --once
```

Or use a GPS driver node that publishes to `/fix`.

### 2. Set Goal via Web Interface

The user selects a goal position in the web interface. The goal is sent to ROS2 via the bridge.

### 3. Route Planning

When both GPS position and goal are available:
1. Planner node receives goal from `/goal_pose` topic
2. Reads current vehicle position from `/fix` topic
3. Plans route using Lanelet2
4. Publishes path to `/planned_path` topic
5. Path is forwarded to web interface for visualization

### 4. Visualization

**RViz:**
```bash
# View the planned path in RViz
ros2 run rviz2 rviz2
# Add display for /planned_path topic
```

**Web Interface:**
- Path is automatically displayed on the map
- Lanelet network is shown from metadata

## OSM File Format for Lanelet2

Lanelet2 expects OSM files with specific tags for lane-level information:

### Required Tags
- `type=lanelet`: Defines a lanelet (lane)
- `subtype`: Type of lanelet (e.g., `road`, `highway`)
- `location`: Indoor/outdoor
- `one_way`: Direction of traffic

### Example Lanelet in OSM
```xml
<way id="1" visible="true">
  <nd ref="101"/>
  <nd ref="102"/>
  <tag k="type" v="line_thin"/>
  <tag k="subtype" v="solid"/>
</way>
<relation id="10" visible="true">
  <member type="way" ref="1" role="left"/>
  <member type="way" ref="2" role="right"/>
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
  <tag k="location" v="urban"/>
  <tag k="one_way" v="yes"/>
</relation>
```

### Converting Standard OSM to Lanelet2 Format

If you have a standard OSM file (roads only), you may need to convert it to Lanelet2 format or use Lanelet2's compatibility mode. Tools are available:
- [Lanelet2 Converter](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_examples)
- JOSM with Lanelet2 plugin

## API Changes

### Metadata Type

The `/map_metadata` topic now publishes Lanelet2 metadata:

```json
{
  "type": "lanelet2",
  "bounds": {
    "min_x": 0.0,
    "max_x": 1000.0,
    "min_y": 0.0,
    "max_y": 1000.0
  },
  "num_lanelets": 150,
  "num_areas": 5,
  "num_regulatory_elements": 20,
  "lanelets": [...]
}
```

### Web API Status

The `/api/status` endpoint now includes:
```json
{
  "map_type": "lanelet2",
  "map_library": "Lanelet2"
}
```

## Traffic Rules

Lanelet2 uses traffic rules to determine valid routing:

**Current Configuration:** German vehicle traffic rules

```python
self.traffic_rules = createTrafficRules("de", "vehicle")
```

**Available Options:**
- Country codes: `"de"` (Germany), `"us"` (USA), etc.
- Participant types: `"vehicle"`, `"pedestrian"`, `"bicycle"`

This can be configured in `lanelet2_map_loader.py` if needed for different regions.

## Troubleshooting

### "No vehicle GPS position available"
- Check that `/fix` topic is being published
- Verify GPS fix status is valid
- Run: `ros2 topic echo /fix`

### "No path found using Lanelet2 routing"
- Ensure start and end positions are within 50m of a lanelet
- Check that lanelets are connected in the map
- Verify routing graph was built successfully
- Check map file format is compatible with Lanelet2

### "Lanelet2 library is not available"
- Ensure Lanelet2 was built successfully in Docker
- Check that Lanelet2 workspace is sourced
- Rebuild Docker container if necessary

### Map Loading Errors
- Verify OSM file is in Lanelet2 format
- Check file path is correct
- Review error logs for specific parsing issues

## Migration Notes

### For Developers

The old OSM loader has been removed. Lanelet2 reads OSM files directly:

1. **Use Lanelet2MapLoader:**
   ```python
   from .lanelet2_map_loader import Lanelet2MapLoader

   # Lanelet2 loads OSM files directly with its native parser
   loader = Lanelet2MapLoader()
   loader.load_osm_file('/path/to/map.osm')
   ```

2. **Update variable names:**
   - `num_nodes, num_ways` → `num_lanelets, num_areas`
   - `osm_loader` → `map_loader`

3. **Add GPS subscription:**
   ```python
   self.gps_sub = self.create_subscription(
       NavSatFix, '/fix', self.gps_callback, 10
   )
   ```

4. **Update path planning to use GPS position:**
   ```python
   start_lat = self.vehicle_gps.latitude
   start_lon = self.vehicle_gps.longitude
   ```

### OSM File Support

**Lanelet2 reads OSM files directly** using its built-in parser:
- The old `osm_map_loader.py` has been removed
- Lanelet2's native parser (`lanelet2.io.load()`) handles OSM format
- Maps must be in Lanelet2 format (OSM with lane-level tags)
- Standard OSM files need conversion to Lanelet2 format

## Performance Considerations

- **Map Loading:** Lanelet2 map loading may take longer for large maps (includes building routing graph)
- **Route Planning:** Generally faster than NetworkX Dijkstra for lane-level routing
- **Memory Usage:** Higher than simple OSM approach due to detailed lane information

## Future Enhancements

Potential improvements for the Lanelet2 integration:

1. **Cost Functions:** Add support for different routing costs (time, distance, comfort)
2. **Lane Changes:** Provide lane-level routing with lane change information
3. **Regulatory Elements:** Display traffic lights and signs in the web interface
4. **Multi-Participant:** Support routing for different vehicle types
5. **Dynamic Routing:** Incorporate real-time traffic information
6. **Map Validation:** Add tools to validate Lanelet2 map correctness

## References

- [Lanelet2 Documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [Lanelet2 Paper](https://arxiv.org/abs/1809.10728)
- [ROS2 sensor_msgs/NavSatFix](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)
- [ROS2 nav_msgs/Path](https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html)

## Support

For issues or questions about the Lanelet2 integration:
1. Check the troubleshooting section above
2. Review Lanelet2 documentation
3. Open an issue on the project repository
