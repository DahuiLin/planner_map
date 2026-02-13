// WebSocket connection
let ws = null;
let reconnectInterval = null;

// Canvas and map state
let canvas, ctx;
let mapData = null;
let currentGoal = null;
let currentTrajectory = null;

// Initialize on page load
document.addEventListener('DOMContentLoaded', function() {
    canvas = document.getElementById('map-canvas');
    ctx = canvas.getContext('2d');
    
    // Setup event listeners
    canvas.addEventListener('click', handleCanvasClick);
    document.getElementById('goal-form').addEventListener('submit', handleGoalSubmit);
    document.getElementById('btn-calculate-spline').addEventListener('click', calculateSplineTrajectory);
    
    // Connect WebSocket
    connectWebSocket();
    
    // Initial status fetch
    updateStatus();
    
    // Periodic status updates
    setInterval(updateStatus, 2000);
    
    // Draw initial canvas
    drawMap();
    
    addLog('Application initialized');
});

// WebSocket functions
function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    addLog('Connecting to WebSocket...');
    
    ws = new WebSocket(wsUrl);
    
    ws.onopen = function() {
        document.getElementById('connection-status').textContent = '🟢 Connected';
        document.getElementById('connection-status').classList.add('connected');
        addLog('WebSocket connected', 'success');
        
        if (reconnectInterval) {
            clearInterval(reconnectInterval);
            reconnectInterval = null;
        }
    };
    
    ws.onmessage = function(event) {
        const message = JSON.parse(event.data);
        handleWebSocketMessage(message);
    };
    
    ws.onclose = function() {
        document.getElementById('connection-status').textContent = '⚫ Disconnected';
        document.getElementById('connection-status').classList.remove('connected');
        addLog('WebSocket disconnected', 'warning');
        
        // Attempt to reconnect
        if (!reconnectInterval) {
            reconnectInterval = setInterval(connectWebSocket, 5000);
        }
    };
    
    ws.onerror = function(error) {
        addLog('WebSocket error', 'error');
    };
}

function handleWebSocketMessage(message) {
    switch(message.type) {
        case 'goal':
            addLog('Goal updated via WebSocket', 'success');
            updateStatus();
            break;
        case 'map':
            addLog('Map updated via WebSocket', 'success');
            fetchMap();
            break;
        case 'trajectory':
            addLog('Spline trajectory updated via WebSocket', 'success');
            fetchTrajectory();
            break;
        case 'pong':
            // Heartbeat response
            break;
    }
}

// API functions
async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        document.getElementById('sys-status').textContent = data.status;
        document.getElementById('map-loaded').textContent = data.has_map ? '✓ Yes' : '✗ No';
        document.getElementById('goal-set').textContent = data.has_goal ? '✓ Yes' : '✗ No';
        document.getElementById('path-length').textContent = data.path_length;
        document.getElementById('trajectory-length').textContent = data.trajectory_length || 0;
        document.getElementById('system-status').textContent = `System: ${data.status}`;
        
    } catch (error) {
        console.error('Error fetching status:', error);
    }
}

async function fetchMap() {
    try {
        const response = await fetch('/api/map');
        const data = await response.json();
        
        if (!data.error) {
            mapData = data;
            document.getElementById('map-dimensions').textContent = 
                `Map: ${data.width}x${data.height} @ ${data.resolution}m/cell`;
            drawMap();
            addLog('Map data loaded', 'success');
        }
    } catch (error) {
        console.error('Error fetching map:', error);
    }
}

async function setGoal(x, y) {
    const goalData = {
        pose: {
            position: { x: x, y: y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
    };

    try {
        const response = await fetch('/api/goal', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(goalData)
        });

        const data = await response.json();

        if (data.status === 'success') {
            currentGoal = goalData.pose;
            displayGoal();
            drawMap();
            addLog(`Goal set: (${x.toFixed(2)}, ${y.toFixed(2)})`, 'success');
            updateStatus();
        }
    } catch (error) {
        console.error('Error setting goal:', error);
        addLog('Error setting goal', 'error');
    }
}

async function fetchTrajectory() {
    try {
        const response = await fetch('/api/trajectory');
        const data = await response.json();

        if (!data.error && data.trajectory) {
            currentTrajectory = data.trajectory;
            drawMap();
            addLog(`Trajectory loaded: ${data.trajectory.length} points`, 'success');
        }
    } catch (error) {
        console.error('Error fetching trajectory:', error);
    }
}

async function calculateSplineTrajectory() {
    try {
        const response = await fetch('/api/trajectory/calculate', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            }
        });

        const data = await response.json();

        if (data.status === 'success') {
            addLog('Spline trajectory calculation triggered', 'success');
        }
    } catch (error) {
        console.error('Error triggering spline calculation:', error);
        addLog('Error triggering spline calculation', 'error');
    }
}

// Canvas drawing functions
function drawMap() {
    // Clear canvas
    ctx.fillStyle = '#f0f0f0';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    if (mapData) {
        // Draw map grid
        const cellWidth = canvas.width / mapData.width;
        const cellHeight = canvas.height / mapData.height;
        
        for (let i = 0; i < mapData.height; i++) {
            for (let j = 0; j < mapData.width; j++) {
                const index = i * mapData.width + j;
                const value = mapData.data[index];
                
                // Color based on occupancy value
                if (value === -1) {
                    ctx.fillStyle = '#888'; // Unknown
                } else if (value === 0) {
                    ctx.fillStyle = '#fff'; // Free
                } else {
                    ctx.fillStyle = '#000'; // Occupied
                }
                
                ctx.fillRect(j * cellWidth, i * cellHeight, cellWidth, cellHeight);
            }
        }
    }
    
    // Draw goal if set
    if (currentGoal) {
        const goalX = (currentGoal.position.x + 2.5) / 5.0 * canvas.width;
        const goalY = (1 - (currentGoal.position.y + 2.5) / 5.0) * canvas.height;

        ctx.fillStyle = '#ff0000';
        ctx.beginPath();
        ctx.arc(goalX, goalY, 10, 0, 2 * Math.PI);
        ctx.fill();

        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.stroke();
    }

    // Draw spline trajectory if available
    if (currentTrajectory && currentTrajectory.length > 1) {
        ctx.strokeStyle = '#00ff00';  // Green for trajectory
        ctx.lineWidth = 3;
        ctx.beginPath();

        for (let i = 0; i < currentTrajectory.length; i++) {
            const point = currentTrajectory[i];
            const x = (point.position.x + 2.5) / 5.0 * canvas.width;
            const y = (1 - (point.position.y + 2.5) / 5.0) * canvas.height;

            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }

        ctx.stroke();

        // Draw points along trajectory
        ctx.fillStyle = '#00ff00';
        for (let i = 0; i < currentTrajectory.length; i += 5) {  // Draw every 5th point
            const point = currentTrajectory[i];
            const x = (point.position.x + 2.5) / 5.0 * canvas.width;
            const y = (1 - (point.position.y + 2.5) / 5.0) * canvas.height;

            ctx.beginPath();
            ctx.arc(x, y, 3, 0, 2 * Math.PI);
            ctx.fill();
        }
    }
    
    // Draw grid lines
    ctx.strokeStyle = '#ddd';
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 10; i++) {
        const x = (i / 10) * canvas.width;
        const y = (i / 10) * canvas.height;
        
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
        
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
    }
}

// Event handlers
function handleCanvasClick(event) {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    // Convert canvas coordinates to map coordinates (-2.5 to 2.5)
    const mapX = (x / canvas.width) * 5.0 - 2.5;
    const mapY = (1 - y / canvas.height) * 5.0 - 2.5;
    
    setGoal(mapX, mapY);
}

function handleGoalSubmit(event) {
    event.preventDefault();
    
    const x = parseFloat(document.getElementById('goal-x').value);
    const y = parseFloat(document.getElementById('goal-y').value);
    
    setGoal(x, y);
}

function displayGoal() {
    if (currentGoal) {
        const goalText = `Position: (${currentGoal.position.x.toFixed(2)}, ${currentGoal.position.y.toFixed(2)})`;
        document.getElementById('current-goal').textContent = goalText;
    } else {
        document.getElementById('current-goal').textContent = 'No goal set';
    }
}

// Utility functions
function refreshMap() {
    fetchMap();
    addLog('Refreshing map...');
}

function clearPath() {
    currentGoal = null;
    displayGoal();
    drawMap();
    addLog('Path cleared');
}

function addLog(message, type = 'info') {
    const logContainer = document.getElementById('log-container');
    const timestamp = new Date().toLocaleTimeString();
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry ${type}`;
    logEntry.textContent = `[${timestamp}] ${message}`;
    
    logContainer.insertBefore(logEntry, logContainer.firstChild);
    
    // Keep only last 50 entries
    while (logContainer.children.length > 50) {
        logContainer.removeChild(logContainer.lastChild);
    }
}
