# Cómo está conectada la Interfaz Web con ROS2

## 🔗 Resumen de la Conexión

La interfaz web y el programa principal de ROS2 se coordinan a través de un **nodo puente (bridge)** llamado `ros2_web_bridge` que actúa como intermediario bidireccional entre ambos sistemas.

## 🏗️ Arquitectura de Comunicación

```
┌─────────────────────────────────────────────────────────────────┐
│                    FLUJO DE COMUNICACIÓN                         │
└─────────────────────────────────────────────────────────────────┘

Usuario → Navegador Web → FastAPI → ROS2 Bridge → ROS2 Nodes
    ↑                         ↓          ↑            ↓
    └─────────────────────────┴──────────┴────────────┘
              Comunicación Bidireccional
```

## 📡 Componentes de la Conexión

### 1. ROS2 Web Bridge (Puente)

**Ubicación**: `ros2_ws/src/planner_map/planner_map/ros2_web_bridge.py`

Este nodo ROS2 actúa como puente entre los dos sistemas:

#### Funciones del Puente:

**A. Dirección ROS2 → Web (Publicación de datos)**
- **Mapas**: Escucha el topic `/map` y envía datos a la API web
- **Rutas**: Escucha el topic `/planned_path` y envía rutas planificadas a la web

**B. Dirección Web → ROS2 (Recepción de comandos)**
- **Objetivos**: Consulta periódicamente la API web buscando nuevos objetivos
- **Publicación**: Publica objetivos recibidos al topic `/goal_pose` de ROS2

### 2. FastAPI (Servidor Web)

**Ubicación**: `web_interface/main.py`

Proporciona la interfaz entre el navegador y ROS2:

#### Endpoints API REST:

| Método | Endpoint | Propósito | Usado Por |
|--------|----------|-----------|-----------|
| `GET` | `/` | Interfaz web principal | Navegador |
| `GET` | `/api/status` | Estado del sistema | Navegador |
| `GET` | `/api/map` | Obtener mapa actual | Navegador |
| `POST` | `/api/map` | Actualizar mapa | ROS2 Bridge |
| `POST` | `/api/goal` | Establecer nuevo objetivo | Navegador |
| `GET` | `/api/goal/latest` | Obtener último objetivo | ROS2 Bridge |
| `GET` | `/api/path` | Obtener ruta actual | Navegador |
| `POST` | `/api/path` | Actualizar ruta | ROS2 Bridge |

#### WebSocket:
- **Endpoint**: `/ws`
- **Propósito**: Actualizaciones en tiempo real al navegador
- **Eventos**: Notificaciones de nuevos mapas, objetivos y rutas

### 3. Navegador Web

**Ubicación**: `web_interface/static/app.js`

El frontend JavaScript se comunica con la API:
- Envía objetivos cuando el usuario hace clic en el mapa
- Recibe actualizaciones en tiempo real vía WebSocket
- Visualiza mapas, rutas y estado del sistema

## 🔄 Flujos de Comunicación Detallados

### Flujo 1: Usuario Establece un Objetivo

```
┌─────────────┐     ┌──────────┐     ┌─────────────┐     ┌──────────┐
│  Navegador  │────▶│ FastAPI  │────▶│ ROS2 Bridge │────▶│  ROS2    │
│   (Click)   │ 1   │ POST goal│ 2   │  Poll goal  │ 3   │/goal_pose│
└─────────────┘     └──────────┘     └─────────────┘     └──────────┘
                          │                                      │
                          │ 4. WebSocket broadcast               │
                          ▼                                      │
                    ┌─────────────┐                              │
                    │ Navegador   │◀─────────────────────────────┘
                    │ (Actualiza) │      5. Planner calcula ruta
                    └─────────────┘
```

**Paso a paso:**
1. **Usuario hace clic** en el mapa web
2. **JavaScript envía** `POST /api/goal` con coordenadas
3. **FastAPI almacena** el objetivo en memoria
4. **FastAPI envía** notificación WebSocket a todos los clientes
5. **ROS2 Bridge consulta** `GET /api/goal/latest` cada 1 segundo
6. **Bridge detecta** nuevo objetivo y lo publica en `/goal_pose`
7. **Planner Node** recibe el objetivo y calcula la ruta

### Flujo 2: ROS2 Publica Mapa

```
┌──────────┐     ┌─────────────┐     ┌──────────┐     ┌─────────────┐
│   ROS2   │────▶│ ROS2 Bridge │────▶│ FastAPI  │────▶│  Navegador  │
│   /map   │ 1   │ map_callback│ 2   │ POST map │ 3   │ WebSocket   │
└──────────┘     └─────────────┘     └──────────┘     └─────────────┘
                                            │
                                            │ 4. Actualiza estado
                                            ▼
                                      ┌─────────────┐
                                      │  Variables  │
                                      │   Globales  │
                                      └─────────────┘
```

**Paso a paso:**
1. **Map Server** publica mapa en topic `/map`
2. **Bridge recibe** mapa vía suscripción ROS2
3. **Bridge convierte** OccupancyGrid a JSON
4. **Bridge envía** `POST /api/map` al servidor web
5. **FastAPI actualiza** variable global `current_map`
6. **FastAPI notifica** clientes vía WebSocket
7. **Navegador recibe** notificación y solicita mapa actualizado

### Flujo 3: ROS2 Calcula Ruta

```
┌──────────┐     ┌─────────────┐     ┌──────────┐     ┌─────────────┐
│   ROS2   │────▶│ ROS2 Bridge │────▶│ FastAPI  │────▶│  Navegador  │
│  /path   │ 1   │path_callback│ 2   │ POST path│ 3   │  Visualiza  │
└──────────┘     └─────────────┘     └──────────┘     └─────────────┘
```

**Paso a paso:**
1. **Planner Node** calcula ruta y la publica en `/planned_path`
2. **Bridge recibe** ruta vía suscripción
3. **Bridge convierte** Path a formato JSON
4. **Bridge envía** `POST /api/path` con los datos
5. **FastAPI actualiza** `current_path`
6. **FastAPI notifica** vía WebSocket
7. **Navegador dibuja** la ruta en el mapa

## ⚙️ Configuración Técnica

### Docker Networking

Los contenedores se comunican a través de una red Docker:

```yaml
# docker-compose.yml
networks:
  planner_network:
    driver: bridge
```

- **Contenedor ROS2**: Nombre `ros2`, accesible en la red interna
- **Contenedor Web**: Nombre `web`, puerto 8000 expuesto
- **URL del Bridge**: `http://web:8000/api` (DNS interno de Docker)

### Frecuencia de Polling

El bridge consulta la API web cada **1 segundo**:

```python
# ros2_web_bridge.py, línea 45
self.timer = self.create_timer(1.0, self.check_web_goals)
```

Esta frecuencia es configurable y puede ajustarse según las necesidades.

### Timeouts

- **Envío de mapa**: 5 segundos
- **Envío de ruta**: 5 segundos  
- **Consulta de objetivos**: 2 segundos

## 🚀 Cómo Iniciar el Sistema Conectado

### Opción 1: Docker Compose (Recomendado)

```bash
# Inicia todos los servicios conectados
docker-compose up --build

# Los tres nodos ROS2 se inician automáticamente:
# - map_server
# - planner_node
# - ros2_web_bridge (el puente)
```

### Opción 2: Manual (Desarrollo)

**Terminal 1 - ROS2:**
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch planner_map planner_map.launch.py
```

**Terminal 2 - Web:**
```bash
cd web_interface
pip install -r requirements.txt
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## 🔍 Verificación de la Conexión

### 1. Verificar que el Bridge está corriendo

```bash
# En el contenedor ROS2
docker-compose exec ros2 bash
ros2 node list

# Deberías ver:
# /map_server
# /planner_node
# /ros2_web_bridge  ← Este es el puente
```

### 2. Verificar Topics ROS2

```bash
ros2 topic list

# Deberías ver:
# /map              ← Map Server publica aquí
# /goal_pose        ← Bridge publica objetivos aquí
# /planned_path     ← Planner publica rutas aquí
```

### 3. Monitorear comunicación del Bridge

```bash
# Ver logs del bridge
docker-compose logs -f ros2

# Busca mensajes como:
# [ros2_web_bridge]: ROS2-Web bridge initialized
# [ros2_web_bridge]: Map data sent to web interface
# [ros2_web_bridge]: Published new goal from web: x=1.50, y=2.30
```

### 4. Verificar API Web

```bash
# Comprobar estado
curl http://localhost:8000/api/status

# Obtener mapa (si está disponible)
curl http://localhost:8000/api/map

# Obtener último objetivo
curl http://localhost:8000/api/goal/latest
```

### 5. Probar envío de objetivo

```bash
# Enviar un objetivo de prueba
curl -X POST http://localhost:8000/api/goal \
  -H "Content-Type: application/json" \
  -d '{
    "pose": {
      "position": {"x": 1.0, "y": 2.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }'

# Luego verifica en los logs de ROS2 que se publicó en /goal_pose
```

## 🐛 Solución de Problemas

### Problema: El Bridge no puede conectar con la API web

**Síntoma**: Logs muestran `Could not reach web API`

**Solución**:
```bash
# Verifica que ambos contenedores estén en la misma red
docker network inspect planner_map_planner_network

# Asegúrate que el contenedor web está corriendo
docker-compose ps

# Prueba conectividad desde el contenedor ROS2
docker-compose exec ros2 ping web
```

### Problema: Objetivos no llegan a ROS2

**Verificación**:
```bash
# Terminal 1: Escucha el topic
ros2 topic echo /goal_pose

# Terminal 2: Envía objetivo desde la web
# (hacer clic en el mapa)

# Si no ves el objetivo, verifica:
docker-compose logs ros2_web_bridge
```

### Problema: Mapas no aparecen en la web

**Verificación**:
```bash
# Verifica que map_server está publicando
ros2 topic hz /map

# Verifica logs del bridge
docker-compose logs -f ros2 | grep "Map data sent"

# Verifica en la API web
curl http://localhost:8000/api/map
```

## 📊 Diagrama de Secuencia Completo

```
┌─────────┐    ┌─────────┐    ┌──────────┐    ┌─────────┐    ┌──────────┐
│ Usuario │    │ Browser │    │ FastAPI  │    │  Bridge │    │   ROS2   │
└────┬────┘    └────┬────┘    └────┬─────┘    └────┬────┘    └────┬─────┘
     │              │              │               │              │
     │ 1. Click Map │              │               │              │
     │─────────────>│              │               │              │
     │              │ 2. POST goal │               │              │
     │              │─────────────>│               │              │
     │              │              │ 3. Store goal │              │
     │              │              │──────────┐    │              │
     │              │              │          │    │              │
     │              │<─────────────│<─────────┘    │              │
     │              │ 4. WebSocket │               │              │
     │              │   broadcast  │               │              │
     │              │              │<──────────────│              │
     │              │              │ 5. Poll goal  │              │
     │              │              │ (every 1s)    │              │
     │              │              │               │              │
     │              │              │───────────────>              │
     │              │              │ 6. Return goal│              │
     │              │              │               │ 7. Pub topic │
     │              │              │               │─────────────>│
     │              │              │               │  /goal_pose  │
     │              │              │               │              │
     │              │              │               │<─────────────│
     │              │              │               │ 8. Sub /map  │
     │              │              │<──────────────│              │
     │              │              │ 9. POST map   │              │
     │              │<─────────────│               │              │
     │<─────────────│10. WebSocket │               │              │
     │ 11. Update UI│              │               │              │
```

## 💡 Mejoras Futuras

Para mejorar el sistema de comunicación:

1. **Usar WebSocket bidireccional**: En lugar de polling, usar WebSocket entre Bridge y FastAPI
2. **Redis/RabbitMQ**: Agregar un message broker para comunicación más robusta
3. **ROS Bridge Library**: Usar rosbridge_server para comunicación WebSocket nativa
4. **REST en ROS2**: Implementar servidor REST directamente en el nodo ROS2

## 📚 Referencias

- **Código del Bridge**: `ros2_ws/src/planner_map/planner_map/ros2_web_bridge.py`
- **API Web**: `web_interface/main.py`
- **Launch File**: `ros2_ws/src/planner_map/launch/planner_map.launch.py`
- **Frontend**: `web_interface/static/app.js`

---

**Resumen**: La interfaz web y ROS2 se conectan mediante el nodo `ros2_web_bridge` que:
1. Suscribe a topics de ROS2 (/map, /planned_path)
2. Envía datos a la API REST de FastAPI
3. Consulta la API para obtener objetivos del usuario
4. Publica esos objetivos en topics de ROS2 (/goal_pose)

Este diseño permite la comunicación bidireccional manteniendo ambos sistemas desacoplados.
