"""
FastAPI Web Interface for ROS2 Planner Map
"""
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
from fastapi import Request
from pydantic import BaseModel
from typing import List, Optional
from contextlib import asynccontextmanager
import json
import asyncio
from datetime import datetime


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager for startup and shutdown"""
    # Startup
    print("FastAPI server started")
    print("Web interface available at http://localhost:8000")
    yield
    # Shutdown
    print("FastAPI server shutting down")


app = FastAPI(title="Planner Map Web Interface", version="0.1.0", lifespan=lifespan)

# Mount static files and templates
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

# Data models
class Position(BaseModel):
    x: float
    y: float
    z: float = 0.0

class Orientation(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

class Pose(BaseModel):
    position: Position
    orientation: Orientation

class GoalRequest(BaseModel):
    pose: Pose

class MapData(BaseModel):
    width: int
    height: int
    resolution: float
    data: List[int]

# Global state
current_map: Optional[MapData] = None
current_goal: Optional[GoalRequest] = None
current_path: List[Pose] = []
osm_metadata: Optional[dict] = None  # Store OSM-specific metadata

# WebSocket manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                # Connection might be closed, log and skip
                pass

manager = ConnectionManager()

# Routes
@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    """Serve the main web interface"""
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api/status")
async def get_status():
    """Get system status"""
    return {
        "status": "running",
        "timestamp": datetime.now().isoformat(),
        "has_map": current_map is not None,
        "has_goal": current_goal is not None,
        "path_length": len(current_path),
        "map_type": osm_metadata.get("type") if osm_metadata else "unknown"
    }

@app.get("/api/map")
async def get_map():
    """Get current map data"""
    if current_map is None:
        return {"error": "No map available"}
    return current_map

@app.post("/api/goal")
async def set_goal(goal: GoalRequest):
    """Set a new goal for the planner"""
    global current_goal
    current_goal = goal
    
    # Broadcast to WebSocket clients
    await manager.broadcast(json.dumps({
        "type": "goal",
        "data": goal.dict()
    }))
    
    return {"status": "success", "goal": goal}

@app.get("/api/goal/latest")
async def get_latest_goal():
    """Get the latest goal (for ROS2 bridge to poll)"""
    if current_goal is None:
        return {"goal": None}
    return {"goal": current_goal.dict(), "timestamp": datetime.now().isoformat()}

@app.get("/api/path")
async def get_path():
    """Get current planned path"""
    return {"path": current_path}

@app.post("/api/path")
async def update_path(path_data: dict):
    """Update the current path (called by ROS2 bridge)"""
    global current_path
    
    # Extract path poses from the request
    if "path" in path_data:
        current_path = path_data["path"]
    
    # Broadcast to WebSocket clients
    await manager.broadcast(json.dumps({
        "type": "path",
        "data": {"path": current_path, "length": len(current_path)}
    }))
    
    return {"status": "success", "path_length": len(current_path)}

@app.post("/api/map")
async def update_map(map_data: MapData):
    """Update the current map (typically called by ROS2 bridge)"""
    global current_map
    current_map = map_data

    # Broadcast to WebSocket clients
    await manager.broadcast(json.dumps({
        "type": "map",
        "data": {"updated": True}
    }))

    return {"status": "success"}

@app.post("/api/map/metadata")
async def update_map_metadata(metadata: dict):
    """Update OSM map metadata"""
    global osm_metadata
    osm_metadata = metadata

    # Broadcast to WebSocket clients
    await manager.broadcast(json.dumps({
        "type": "map_metadata",
        "data": metadata
    }))

    return {"status": "success"}

@app.get("/api/map/osm")
async def get_osm_map():
    """Get OSM-specific map data"""
    if osm_metadata is None:
        return {"error": "No OSM map loaded"}
    return osm_metadata

@app.post("/api/route")
async def request_route(route_request: dict):
    """Request a route between two points"""
    # This endpoint can be called by the web interface
    # to request a route from ROS2
    # Format: {"start": {"lat": ..., "lon": ...}, "end": {"lat": ..., "lon": ...}}

    # For now, we'll just set the goal and let ROS2 handle it
    # In a full implementation, this would trigger route calculation

    end = route_request.get("end")
    if not isinstance(end, dict):
        return {"error": "Invalid route request: 'end' must be an object"}

    # Prefer explicit x/y if provided (backwards compatibility)
    x = end.get("x")
    y = end.get("y")

    # If x/y are not provided, fall back to documented lat/lon format
    if x is None or y is None:
        lat = end.get("lat")
        lon = end.get("lon")
        if lat is None or lon is None:
            return {
                "error": "Invalid route request: provide either 'x'/'y' or 'lat'/'lon' in 'end'"
            }
        try:
            # Placeholder conversion: map lat/lon to x/y directly.
            # A real implementation would project these into the map frame.
            x = float(lon)
            y = float(lat)
        except (TypeError, ValueError):
            return {"error": "Invalid route request: 'lat' and 'lon' must be numeric"}

    goal = GoalRequest(pose=Pose(
        position=Position(
            x=x,
            y=y,
            z=0.0
        ),
        orientation=Orientation()
    ))
    return await set_goal(goal)
    return {"error": "Invalid route request"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time updates"""
    await manager.connect(websocket)
    try:
        while True:
            # Receive messages from client
            data = await websocket.receive_text()
            message = json.loads(data)
            
            # Handle different message types
            if message.get("type") == "ping":
                await websocket.send_text(json.dumps({"type": "pong"}))
            
    except KeyboardInterrupt:
        pass
    finally:
        manager.disconnect(websocket)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
