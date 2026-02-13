# Connection Diagram: Web Interface ↔ ROS2

## Complete Communication Flow

```
╔═══════════════════════════════════════════════════════════════════════╗
║                   SISTEMA DE COMUNICACIÓN COMPLETO                     ║
╚═══════════════════════════════════════════════════════════════════════╝

┌─────────────────────────────────────────────────────────────────────┐
│                          USUARIO FINAL                               │
│                        (Navegador Web)                               │
└────────────┬────────────────────────────────────────┬───────────────┘
             │                                        │
             │ HTTP/WebSocket                         │
             │ http://localhost:8000                  │
             │                                        │
┌────────────▼────────────────────────────────────────▼───────────────┐
│                      CONTENEDOR WEB                                  │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │                    FASTAPI SERVER                              │ │
│  │                  (web_interface/main.py)                       │ │
│  ├────────────────────────────────────────────────────────────────┤ │
│  │  REST API Endpoints:                                           │ │
│  │  • GET  /                    → Serve HTML interface            │ │
│  │  • GET  /api/status          → System status                   │ │
│  │  • POST /api/goal            ← Receive goals from browser      │ │
│  │  • GET  /api/goal/latest     → Provide goal to ROS2 bridge     │ │
│  │  • POST /api/map             ← Receive map from ROS2 bridge    │ │
│  │  • GET  /api/map             → Provide map to browser          │ │
│  │  • POST /api/path            ← Receive path from ROS2 bridge   │ │
│  │  • GET  /api/path            → Provide path to browser         │ │
│  │  • WS   /ws                  ↔ Real-time updates               │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                          Port: 8000                                  │
└─────────────┬──────────────────────────────────────┬────────────────┘
              │                                      │
              │ HTTP API Calls                       │
              │ http://web:8000/api                  │
              │                                      │
┌─────────────▼──────────────────────────────────────▼────────────────┐
│                      CONTENEDOR ROS2                                 │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │                    ROS2 WEB BRIDGE                             │ │
│  │           (ros2_web_bridge.py) - NODO PUENTE                   │ │
│  ├────────────────────────────────────────────────────────────────┤ │
│  │  Funciones:                                                    │ │
│  │  1. Subscribes to ROS2 topics:                                 │ │
│  │     • /map           → Forward to FastAPI (POST /api/map)      │ │
│  │     • /planned_path  → Forward to FastAPI (POST /api/path)     │ │
│  │                                                                │ │
│  │  2. Polls FastAPI every 1 second:                              │ │
│  │     • GET /api/goal/latest → Get new goals                     │ │
│  │                                                                │ │
│  │  3. Publishes to ROS2 topics:                                  │ │
│  │     • /goal_pose     ← Publish goals from web                  │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                              ▲  │                                    │
│                              │  │                                    │
│                              │  ▼                                    │
│  ┌─────────────────────┐   ┌──────────────────┐                    │
│  │    MAP SERVER       │   │  PLANNER NODE    │                    │
│  │  (map_server.py)    │   │ (planner_node.py)│                    │
│  ├─────────────────────┤   ├──────────────────┤                    │
│  │ Publishes:          │   │ Subscribes:      │                    │
│  │ • /map              │   │ • /goal_pose     │                    │
│  │   (OccupancyGrid)   │   │ • /map           │                    │
│  │                     │   │                  │                    │
│  │                     │   │ Publishes:       │                    │
│  │                     │   │ • /planned_path  │                    │
│  │                     │   │ • /cmd_vel       │                    │
│  └─────────────────────┘   └──────────────────┘                    │
│                                                                      │
│                     ROS2 TOPICS (DDS Middleware)                     │
│  /map  /goal_pose  /planned_path  /cmd_vel                          │
└──────────────────────────────────────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════════

FLUJO 1: Usuario establece objetivo (Web → ROS2)
─────────────────────────────────────────────────────────

1. Usuario hace clic en el mapa
2. app.js → POST /api/goal {x, y, z}
3. FastAPI almacena objetivo en current_goal
4. FastAPI → WebSocket broadcast "goal updated"
5. ROS2 Bridge → GET /api/goal/latest (polling cada 1s)
6. Bridge detecta nuevo objetivo
7. Bridge → Publica PoseStamped en /goal_pose
8. Planner Node → Recibe objetivo
9. Planner → Calcula ruta
10. Planner → Publica Path en /planned_path

═══════════════════════════════════════════════════════════════════════

FLUJO 2: ROS2 publica mapa (ROS2 → Web)
─────────────────────────────────────────────────────────

1. Map Server → Publica OccupancyGrid en /map
2. ROS2 Bridge → Recibe via suscripción /map
3. Bridge → Convierte OccupancyGrid a JSON
4. Bridge → POST /api/map {width, height, data}
5. FastAPI → Actualiza current_map
6. FastAPI → WebSocket broadcast "map updated"
7. Navegador → GET /api/map
8. app.js → Renderiza mapa en canvas

═══════════════════════════════════════════════════════════════════════

FLUJO 3: ROS2 publica ruta planificada (ROS2 → Web)
─────────────────────────────────────────────────────────

1. Planner Node → Calcula ruta
2. Planner → Publica Path en /planned_path
3. ROS2 Bridge → Recibe via suscripción /planned_path
4. Bridge → Convierte Path a JSON array
5. Bridge → POST /api/path {path: [...]}
6. FastAPI → Actualiza current_path
7. FastAPI → WebSocket broadcast "path updated"
8. Navegador → Recibe notificación
9. app.js → Dibuja ruta en el mapa

═══════════════════════════════════════════════════════════════════════

CARACTERÍSTICAS CLAVE:
─────────────────────────────────────────────────────────

✓ Comunicación bidireccional completa
✓ Desacoplamiento entre sistemas (pueden ejecutarse independientemente)
✓ Actualizaciones en tiempo real via WebSocket
✓ Polling inteligente con throttling de errores
✓ Manejo robusto de errores y timeouts
✓ Todos los nodos se lanzan automáticamente con docker-compose

═══════════════════════════════════════════════════════════════════════

DOCKER NETWORKING:
─────────────────────────────────────────────────────────

Ambos contenedores están en la red: planner_network
- Contenedor "ros2" puede acceder a "web" via DNS interno
- URL del bridge: http://web:8000/api
- Puerto expuesto al host: 8000 (solo web)

═══════════════════════════════════════════════════════════════════════
```

## Archivos Clave

| Archivo | Propósito |
|---------|-----------|
| `ros2_ws/src/planner_map/planner_map/ros2_web_bridge.py` | Nodo puente ROS2 ↔ Web |
| `web_interface/main.py` | Servidor FastAPI con REST API |
| `web_interface/static/app.js` | Cliente JavaScript (frontend) |
| `ros2_ws/src/planner_map/launch/planner_map.launch.py` | Launch incluye el bridge |
| `CONEXION.md` | Documentación completa en español |

## Comandos de Verificación

```bash
# Ver nodos activos (debe incluir ros2_web_bridge)
docker-compose exec ros2 ros2 node list

# Ver topics (debe incluir /map, /goal_pose, /planned_path)
docker-compose exec ros2 ros2 topic list

# Monitorear comunicación del bridge
docker-compose logs -f ros2 | grep bridge

# Verificar API web
curl http://localhost:8000/api/status

# Enviar objetivo de prueba
curl -X POST http://localhost:8000/api/goal \
  -H "Content-Type: application/json" \
  -d '{"pose":{"position":{"x":1.0,"y":2.0,"z":0.0},"orientation":{"x":0,"y":0,"z":0,"w":1}}}'
```

## Para Más Información

Ver `CONEXION.md` para documentación completa en español con diagramas de secuencia, solución de problemas y ejemplos detallados.
