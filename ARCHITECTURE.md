# Planner Map - Architecture Overview

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Docker Compose Network                    │
│                                                              │
│  ┌─────────────────────────┐    ┌────────────────────────┐ │
│  │   ROS2 Container        │    │   Web Container        │ │
│  │                         │    │                        │ │
│  │  ┌──────────────────┐   │    │  ┌─────────────────┐  │ │
│  │  │  planner_node    │   │    │  │  FastAPI Server │  │ │
│  │  │  - Path planning │   │    │  │  - REST API     │  │ │
│  │  │  - Goal handling │   │    │  │  - WebSocket    │  │ │
│  │  └──────────────────┘   │    │  └─────────────────┘  │ │
│  │                         │    │                        │ │
│  │  ┌──────────────────┐   │    │  ┌─────────────────┐  │ │
│  │  │  map_server      │   │    │  │  Static Files   │  │ │
│  │  │  - Map publishing│   │    │  │  - HTML/CSS/JS  │  │ │
│  │  │  - OccupancyGrid │   │    │  └─────────────────┘  │ │
│  │  └──────────────────┘   │    │                        │ │
│  │                         │    │  Port: 8000            │ │
│  │  ┌──────────────────┐   │    │                        │ │
│  │  │  ros2_web_bridge │◄──┼────┤  HTTP Client          │ │
│  │  │  - ROS2 ↔ Web    │   │    │                        │ │
│  │  └──────────────────┘   │    └────────────────────────┘ │
│  │                         │              ▲                 │
│  │  ROS2 Topics:           │              │                 │
│  │  - /map                 │              │ HTTP/WS         │
│  │  - /goal_pose           │              │                 │
│  │  - /planned_path        │              │                 │
│  │  - /cmd_vel             │              │                 │
│  └─────────────────────────┘              │                 │
│                                           │                 │
└───────────────────────────────────────────┼─────────────────┘
                                            │
                                            ▼
                                    ┌───────────────┐
                                    │   Web Browser │
                                    │               │
                                    │  - Map View   │
                                    │  - Controls   │
                                    │  - Status     │
                                    └───────────────┘
```

## 📦 Components

### 1. ROS2 Nodes

#### planner_node
- **Purpose**: Path planning and navigation control
- **Subscribes to**:
  - `/goal_pose` (geometry_msgs/PoseStamped): Target goals
  - `/map` (nav_msgs/OccupancyGrid): Current map
- **Publishes to**:
  - `/planned_path` (nav_msgs/Path): Computed path
  - `/cmd_vel` (geometry_msgs/Twist): Velocity commands

#### map_server
- **Purpose**: Publish and manage occupancy grid maps
- **Publishes to**:
  - `/map` (nav_msgs/OccupancyGrid): Map data
- **Features**:
  - Configurable resolution
  - Periodic publishing
  - Support for static maps

#### ros2_web_bridge
- **Purpose**: Bridge between ROS2 and web interface
- **Functions**:
  - Forward map updates to web API
  - Forward path data to web API
  - Receive goals from web interface
  - Publish goals to ROS2 topics

### 2. Web Interface (FastAPI)

#### Backend Endpoints

**REST API**:
- `GET /` - Main web interface
- `GET /api/status` - System status
- `GET /api/map` - Current map data
- `POST /api/goal` - Set navigation goal
- `GET /api/path` - Get planned path
- `POST /api/map` - Update map (from ROS2)

**WebSocket**:
- `WS /ws` - Real-time bidirectional communication
  - Goal updates
  - Map changes
  - System events

#### Frontend Features

**Map Visualization**:
- Interactive canvas-based map display
- Click-to-set goal functionality
- Visual goal marker
- Grid overlay

**Control Panel**:
- System status display
- Manual goal input form
- Current goal information
- Action buttons

**Real-time Updates**:
- WebSocket connection for live data
- Automatic reconnection
- Activity logging

## 🔄 Data Flow

### Setting a Goal

```
User clicks map
    ↓
JavaScript captures coordinates
    ↓
POST /api/goal (FastAPI)
    ↓
WebSocket broadcast to clients
    ↓
ros2_web_bridge receives goal (future implementation)
    ↓
Publishes to /goal_pose topic
    ↓
planner_node receives goal
    ↓
Plans path
    ↓
Publishes to /planned_path
    ↓
ros2_web_bridge forwards to web API
    ↓
Frontend updates visualization
```

### Map Updates

```
map_server publishes to /map
    ↓
ros2_web_bridge receives map
    ↓
POST /api/map (FastAPI)
    ↓
WebSocket broadcast to clients
    ↓
Frontend fetches map data
    ↓
Renders on canvas
```

## 🐳 Docker Architecture

### Containers

1. **ros2** (Dockerfile.ros)
   - Base: osrf/ros:humble-desktop
   - ROS2 workspace with colcon build
   - All ROS2 nodes
   - Network: planner_network

2. **web** (Dockerfile.web)
   - Base: python:3.11-slim
   - FastAPI application
   - Static file serving
   - Network: planner_network
   - Exposed: Port 8000

### Networking

- Bridge network: `planner_network`
- Internal DNS: Services communicate by name
  - `ros2` → `web` via HTTP
  - `web` → `ros2` (future ROS2 client)

### Volumes

- `./ros2_ws:/workspace/ros2_ws` - Live code updates
- `./web_interface:/app` - Hot reload for web
- `./config:/workspace/config` - Configuration files

## 🚀 Deployment Options

### Development
```bash
docker-compose -f docker-compose.dev.yml up
```
- Hot reload enabled
- Volume mounts for live editing
- Debug-friendly configuration

### Production
```bash
docker-compose up -d
```
- Optimized builds
- Restart policies
- Background execution

## 📊 Technology Stack

**ROS2**:
- ROS2 Humble LTS
- Python 3.10+
- rclpy
- nav2_msgs, geometry_msgs

**Web**:
- FastAPI (async web framework)
- Uvicorn (ASGI server)
- Jinja2 (templating)
- WebSockets

**Frontend**:
- Vanilla JavaScript
- HTML5 Canvas
- CSS3 (Flexbox/Grid)
- WebSocket API

**Infrastructure**:
- Docker
- Docker Compose
- Make (task automation)

## 🔐 Security Considerations

- No authentication implemented (add for production)
- CORS configured for localhost
- WebSocket origin validation needed
- Environment variable management for secrets
- Network isolation via Docker networks

## 📈 Future Enhancements

- [ ] Add authentication/authorization
- [ ] Implement actual path planning algorithms (A*, RRT)
- [ ] 3D visualization support
- [ ] Multi-robot support
- [ ] Cloud deployment configuration
- [ ] Metrics and monitoring
- [ ] Database for map persistence
- [ ] ROS2 parameter reconfiguration via web
- [ ] Save/load map functionality
- [ ] Obstacle detection integration
