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
import json
import asyncio
from datetime import datetime

app = FastAPI(title="Planner Map Web Interface", version="0.1.0")

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
            except:
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
        "path_length": len(current_path)
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

@app.get("/api/path")
async def get_path():
    """Get current planned path"""
    return {"path": current_path}

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
            
    except WebSocketDisconnect:
        manager.disconnect(websocket)

@app.on_event("startup")
async def startup_event():
    """Initialize on startup"""
    print("FastAPI server started")
    print("Web interface available at http://localhost:8000")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    print("FastAPI server shutting down")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
