# Spline Trajectory Generation

## Overview

The planner_map system now includes **smooth spline trajectory generation** using cubic B-spline interpolation. This feature generates continuous, smooth trajectories from discrete waypoints obtained through Lanelet2 routing, with configurable velocity constraints to ensure vehicle safety.

## Features

- **Cubic B-spline interpolation**: Generates smooth, continuous trajectories from waypoints
- **Velocity constraints**: Ensures trajectory velocity never exceeds specified maximum
- **Configurable sampling**: Trajectory sampled at configurable time intervals (dt)
- **Web interface control**: User can trigger calculation via button click
- **Real-time visualization**: Trajectory displayed as green line on web interface map
- **ROS2 topic publishing**: Trajectory published for downstream motion controllers

## How It Works

### 1. Route Planning with Lanelet2

First, the system uses Lanelet2 to find an optimal route from the vehicle's current position (GPS) to a user-selected destination:

```
Vehicle GPS → Lanelet2 Routing → Waypoints → /planned_path topic
```

### 2. Spline Trajectory Calculation

When the user clicks **"Calculate Spline Trajectory"** in the web interface:

1. Web interface sends trigger to FastAPI backend (`POST /api/trajectory/calculate`)
2. ROS2-web bridge polls the trigger and publishes to `/calculate_spline_trajectory` topic
3. Planner node receives trigger and processes current waypoints:
   - Extracts waypoints from `/planned_path`
   - Uses `SplineTrajectoryGenerator` to create smooth trajectory
   - Applies cubic B-spline interpolation
   - Samples trajectory at `dt` intervals
   - Enforces `max_velocity` constraint
4. Publishes sampled trajectory to `/spline_trajectory` topic
5. ROS2-web bridge forwards trajectory to web interface
6. Web interface displays trajectory as green line on map

## Configuration Parameters

### ROS2 Parameters (planner_node)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory_dt` | float | 0.1 | Sampling time interval in seconds (e.g., 0.1s = 10Hz) |
| `max_velocity` | float | 5.0 | Maximum allowed velocity in m/s |

### Example Launch Configuration

```python
# In launch file
Node(
    package='planner_map',
    executable='planner_node',
    parameters=[
        {'osm_file': '/path/to/map.osm'},
        {'trajectory_dt': 0.1},      # Sample every 0.1 seconds
        {'max_velocity': 5.0}        # Max velocity 5 m/s
    ]
)
```

Or via command line:

```bash
ros2 run planner_map planner_node \
    --ros-args \
    -p trajectory_dt:=0.1 \
    -p max_velocity:=5.0 \
    -p osm_file:=/path/to/map.osm
```

## ROS2 Topics

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/spline_trajectory` | `nav_msgs/Path` | Sampled spline trajectory with position points |
| `/planned_path` | `nav_msgs/Path` | Original waypoints from Lanelet2 routing |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/calculate_spline_trajectory` | `std_msgs/String` | Trigger to calculate spline from current path |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Destination goal for route planning |
| `/fix` | `sensor_msgs/NavSatFix` | Vehicle GPS position (route start point) |

## Web Interface

### User Workflow

1. **Set destination goal**:
   - Click on the map canvas, OR
   - Enter X/Y coordinates manually

2. **Wait for route calculation**:
   - System automatically plans route using Lanelet2
   - Waypoints displayed (if visualization enabled)

3. **Calculate spline trajectory**:
   - Click **"Calculate Spline Trajectory"** button
   - System generates smooth trajectory
   - Green line appears on map showing trajectory

4. **Verify trajectory**:
   - Visual confirmation that trajectory stays within road boundaries
   - Check trajectory length in status panel

### Visual Indicators

- **Red circle**: Goal position
- **Green line**: Spline trajectory (smooth, continuous)
- **Green dots**: Sample points along trajectory (shown every 5th point)

## API Endpoints

### REST API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `POST /api/trajectory/calculate` | POST | Trigger spline trajectory calculation |
| `GET /api/trajectory` | GET | Retrieve current spline trajectory |
| `POST /api/trajectory` | POST | Update trajectory (called by ROS2 bridge) |
| `GET /api/trajectory/trigger` | GET | Poll trigger status (ROS2 bridge polls this) |

### WebSocket Messages

Real-time updates via WebSocket (`/ws`):

```javascript
// Trajectory update
{
    "type": "trajectory",
    "data": {
        "trajectory": [...],  // Array of trajectory points
        "length": 150         // Number of points
    }
}
```

## Algorithm Details

### Cubic B-Spline Interpolation

The system uses **scipy.interpolate.splprep** with the following configuration:

- **Degree**: k=3 (cubic splines)
- **Smoothing**: s=0 (exact interpolation through waypoints)
- **Parameterization**: Cumulative distance along waypoints

### Velocity Constraint

The trajectory velocity is constrained using time-distance parameterization:

1. Calculate total path distance
2. Determine total time: `T = distance / max_velocity`
3. Sample trajectory at uniform time intervals: `t = [0, dt, 2*dt, ..., T]`
4. Ensure no instantaneous velocity exceeds `max_velocity`

### Fallback Behavior

- **< 2 waypoints**: No trajectory generated (warning logged)
- **2 waypoints**: Linear interpolation between points
- **≥ 3 waypoints**: Cubic B-spline interpolation

## Testing

A test suite is provided to validate the spline trajectory generator:

```bash
# Run test suite
python3 test_spline_trajectory.py
```

### Test Cases

1. **Simple straight line**: 2 waypoints, validates basic interpolation
2. **L-shaped path**: 5 waypoints, validates curve handling
3. **Complex S-curve**: 9 waypoints, validates smooth transitions

Expected output:
```
✓ All trajectories respect velocity constraints
✓ Trajectories are smooth and continuous
✓ Sample points correctly distributed at dt intervals
```

## Integration with Motion Controllers

The `/spline_trajectory` topic publishes `nav_msgs/Path` messages that can be consumed by motion controllers:

```python
# Example subscriber in a motion controller
self.trajectory_sub = self.create_subscription(
    Path,
    '/spline_trajectory',
    self.trajectory_callback,
    10
)

def trajectory_callback(self, msg):
    # Extract trajectory points
    for pose in msg.poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        # Use for path following control
```

## Troubleshooting

### No trajectory generated

**Symptoms**: Button clicked but no green line appears

**Possible causes**:
- No path available (set a goal first)
- Path has < 2 waypoints (check `/planned_path` topic)
- Scipy not installed (check container logs)

**Solution**:
```bash
# Check path availability
ros2 topic echo /planned_path --once

# Check spline trigger
ros2 topic echo /calculate_spline_trajectory

# Check trajectory output
ros2 topic echo /spline_trajectory
```

### Trajectory exceeds velocity constraint

**Symptoms**: Warning in logs about invalid trajectory

**Possible causes**:
- Waypoints too far apart
- dt too small
- max_velocity too low

**Solution**:
- Increase `max_velocity` parameter
- Increase `trajectory_dt` parameter
- Check waypoint spacing

### Dependencies missing

**Symptoms**: ModuleNotFoundError for numpy or scipy

**Solution**:
```bash
# Install in ROS2 container
pip3 install numpy scipy

# Or rebuild Docker container (scipy added to requirements.txt)
docker-compose build ros2
```

## Future Enhancements

Potential improvements for future development:

1. **Acceleration constraints**: Limit trajectory acceleration in addition to velocity
2. **Jerk minimization**: Optimize for smoother motion
3. **Dynamic obstacles**: Real-time trajectory replanning
4. **Multiple trajectory options**: Generate and compare multiple trajectories
5. **Trajectory editing**: Allow user to adjust trajectory via web interface
6. **Cost optimization**: Include trajectory smoothness in cost function
7. **Lane keeping**: Additional constraint to stay within lane boundaries

## References

- **Scipy B-spline documentation**: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splprep.html
- **Lanelet2 routing**: See `LANELET2_INTEGRATION.md`
- **ROS2 Path message**: http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html

## License

Same as parent project (see root LICENSE file)
