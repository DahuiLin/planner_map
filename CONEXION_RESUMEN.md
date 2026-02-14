# Resumen Rápido: Conexión Web ↔ ROS2

## ¿Cómo están conectados?

La interfaz web y ROS2 se comunican a través de un **nodo puente** llamado `ros2_web_bridge`.

## Diagrama Simple

```
┌──────────────┐         ┌───────────┐         ┌──────────────┐
│   USUARIO    │────────▶│  FastAPI  │◀───────│  ROS2 Bridge │
│  (Navegador) │         │  Web API  │         │    (Nodo)    │
└──────────────┘         └───────────┘         └──────┬───────┘
      ▲                        ▲                       │
      │                        │                       ▼
      │                        │                 ┌────────────┐
      └────────────────────────┴─────────────────│ ROS2 Nodes │
              WebSocket                          │ /map       │
           (Tiempo Real)                         │ /goal_pose │
                                                 │ /path      │
                                                 └────────────┘
```

## 3 Componentes Principales

### 1. **FastAPI Web Server** (`web_interface/main.py`)
   - Servidor web con API REST
   - Recibe clicks del usuario
   - Almacena objetivos, mapas y rutas
   - Envía actualizaciones en tiempo real vía WebSocket

### 2. **ROS2 Web Bridge** (`ros2_web_bridge.py`)
   - Nodo ROS2 que conecta ambos mundos
   - **Lee de ROS2**: Subscribe a `/map` y `/planned_path`
   - **Escribe a ROS2**: Publica en `/goal_pose`
   - **Lee de Web**: Consulta API cada 1 segundo para obtener objetivos
   - **Escribe a Web**: Envía mapas y rutas a la API

### 3. **Navegador Web** (JavaScript)
   - Interfaz visual para el usuario
   - Envía objetivos cuando el usuario hace clic
   - Recibe y visualiza mapas y rutas

## Flujo Completo de un Objetivo

```
1. Usuario hace clic en el mapa
   ↓
2. JavaScript captura coordenadas (x, y)
   ↓
3. JavaScript → POST http://localhost:8000/api/goal
   ↓
4. FastAPI guarda el objetivo
   ↓
5. Bridge consulta GET /api/goal/latest (cada 1 segundo)
   ↓
6. Bridge detecta nuevo objetivo
   ↓
7. Bridge convierte a mensaje ROS2 (PoseStamped)
   ↓
8. Bridge publica en topic /goal_pose
   ↓
9. Planner Node recibe el objetivo
   ↓
10. Planner calcula la ruta
   ↓
11. Planner publica en topic /planned_path
   ↓
12. Bridge recibe la ruta
   ↓
13. Bridge convierte a JSON
   ↓
14. Bridge → POST /api/path
   ↓
15. FastAPI envía notificación WebSocket
   ↓
16. Navegador actualiza visualización ✓
```

## Endpoints API

| Endpoint | Método | Usado Por | Propósito |
|----------|--------|-----------|-----------|
| `/api/goal` | POST | Navegador | Enviar objetivo del usuario |
| `/api/goal/latest` | GET | ROS2 Bridge | Obtener último objetivo |
| `/api/map` | POST | ROS2 Bridge | Enviar mapa desde ROS2 |
| `/api/map` | GET | Navegador | Obtener mapa para visualizar |
| `/api/path` | POST | ROS2 Bridge | Enviar ruta planificada |
| `/api/path` | GET | Navegador | Obtener ruta para dibujar |
| `/ws` | WebSocket | Navegador | Actualizaciones en tiempo real |

## Cómo Probar

### 1. Iniciar el Sistema
```bash
docker compose up --build
```

### 2. Abrir el Navegador
```
http://localhost:8000
```

### 3. Hacer Clic en el Mapa
- Haz clic en cualquier parte del mapa
- Verás un punto rojo (el objetivo)

### 4. Ver los Logs
En otra terminal:
```bash
# Ver logs del bridge
docker compose logs -f ros2 | grep bridge

# Deberías ver algo como:
# [ros2_web_bridge]: Published new goal from web: x=1.50, y=2.30
```

### 5. Verificar Topics ROS2
```bash
docker compose exec ros2 ros2 topic echo /goal_pose
```

## Archivos Importantes

```
planner_map/
├── ros2_ws/src/planner_map/
│   ├── planner_map/
│   │   └── ros2_web_bridge.py    ← El puente (CLAVE)
│   └── launch/
│       └── planner_map.launch.py ← Lanza el bridge
│
├── web_interface/
│   ├── main.py                    ← API REST (CLAVE)
│   └── static/
│       └── app.js                 ← Frontend JavaScript
│
├── CONEXION.md                     ← Documentación completa
└── CONNECTION_DIAGRAM.md           ← Diagramas visuales
```

## Preguntas Frecuentes

**P: ¿Por qué usar polling en lugar de WebSocket?**
R: ROS2 no puede usar WebSocket fácilmente. El polling cada 1 segundo es suficientemente rápido para objetivos del usuario y simple de implementar.

**P: ¿Se pueden comunicar directamente sin el bridge?**
R: No. ROS2 usa DDS (Data Distribution Service) y el navegador usa HTTP/WebSocket. El bridge traduce entre ambos protocolos.

**P: ¿Qué pasa si el web server no está disponible?**
R: El bridge maneja errores silenciosamente (solo registra cada 10 intentos) y ROS2 sigue funcionando normalmente.

**P: ¿Puedo agregar más datos?**
R: Sí. Solo necesitas:
1. Agregar endpoint en `web_interface/main.py`
2. Agregar callback en `ros2_web_bridge.py`
3. Actualizar el frontend si es necesario

## Siguiente Paso

Lee [CONEXION.md](CONEXION.md) para información detallada sobre:
- Flujos de comunicación completos
- Configuración avanzada
- Solución de problemas
- Diagramas de secuencia
- Ejemplos de uso

---

**Resumen en una línea:**  
El `ros2_web_bridge` lee topics ROS2 y los envía a FastAPI, y lee la API FastAPI y publica en topics ROS2. ✨
