# Planner Map - ROS2 + FastAPI + Docker

Sistema de planificación y mapeo integrado con ROS2, interfaz web FastAPI y despliegue automatizado con Docker Compose.

## 📋 Descripción

Este proyecto combina:
- **ROS2 Humble**: Sistema de planificación de rutas y servidor de mapas
- **FastAPI**: Interfaz web moderna para visualización y control
- **Docker Compose**: Despliegue automatizado de todos los servicios

## 🏗️ Estructura del Proyecto

```
planner_map/
├── ros2_ws/                    # ROS2 Workspace
│   └── src/
│       └── planner_map/        # Paquete ROS2
│           ├── planner_map/    # Código Python
│           │   ├── planner_node.py     # Nodo de planificación
│           │   └── map_server.py       # Servidor de mapas
│           ├── launch/         # Archivos launch
│           ├── config/         # Configuración
│           ├── package.xml     # Dependencias ROS2
│           └── setup.py        # Setup Python
├── web_interface/              # Interfaz Web FastAPI
│   ├── main.py                # Aplicación FastAPI
│   ├── static/                # CSS y JavaScript
│   ├── templates/             # HTML templates
│   └── requirements.txt       # Dependencias Python
├── docker/                     # Scripts Docker
├── config/                     # Configuración general
├── Dockerfile.ros             # Docker para ROS2
├── Dockerfile.web             # Docker para Web
├── docker-compose.yml         # Orquestación de servicios
└── README.md                  # Este archivo
```

## 🚀 Inicio Rápido

### Prerequisitos

- Docker (>= 20.10)
- Docker Compose (>= 2.0)

### Despliegue con Docker Compose

1. **Clonar el repositorio**:
   ```bash
   git clone https://github.com/DahuiLin/planner_map.git
   cd planner_map
   ```

2. **Construir y ejecutar los servicios**:
   ```bash
   docker-compose up --build
   ```

3. **Acceder a la interfaz web**:
   - Abrir navegador en: `http://localhost:8000`

### Servicios Disponibles

- **Web Interface**: http://localhost:8000
  - Visualización del mapa
  - Control de objetivos de navegación
  - Estado del sistema en tiempo real
  
- **ROS2 Nodes**:
  - `planner_node`: Nodo de planificación de rutas
  - `map_server`: Servidor de mapas

## 🛠️ Desarrollo Local

### Sin Docker - ROS2

1. **Instalar ROS2 Humble**:
   ```bash
   # Seguir instrucciones oficiales de ROS2
   ```

2. **Construir el workspace**:
   ```bash
   cd ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Ejecutar nodos**:
   ```bash
   # Terminal 1 - Servidor de mapas
   ros2 run planner_map map_server
   
   # Terminal 2 - Nodo de planificación
   ros2 run planner_map planner_node
   
   # O usar launch file
   ros2 launch planner_map planner_map.launch.py
   ```

### Sin Docker - Web Interface

1. **Instalar dependencias**:
   ```bash
   cd web_interface
   pip install -r requirements.txt
   ```

2. **Ejecutar servidor**:
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

## 📡 API Endpoints

### REST API

- `GET /`: Interfaz web principal
- `GET /api/status`: Estado del sistema
- `GET /api/map`: Datos del mapa actual
- `POST /api/goal`: Establecer objetivo de navegación
- `GET /api/path`: Obtener ruta planificada
- `POST /api/map`: Actualizar mapa (desde ROS2)

### WebSocket

- `WS /ws`: Conexión WebSocket para actualizaciones en tiempo real

## 🎯 Características

### ROS2
- ✅ Nodo de planificación de rutas
- ✅ Servidor de mapas con OccupancyGrid
- ✅ Publishers y Subscribers configurados
- ✅ Launch files para inicio automático
- ✅ Parámetros configurables

### Web Interface
- ✅ Visualización de mapa interactiva
- ✅ Establecimiento de objetivos por clic
- ✅ Formulario manual de coordenadas
- ✅ WebSocket para actualizaciones en tiempo real
- ✅ Panel de control con estado del sistema
- ✅ Registro de actividad
- ✅ Diseño responsive

### Docker
- ✅ Contenedor ROS2 con todas las dependencias
- ✅ Contenedor Web independiente
- ✅ Networking entre servicios
- ✅ Volumes para desarrollo en caliente
- ✅ Restart automático

## 🔧 Configuración

### Variables de Entorno

Editar `docker-compose.yml` para ajustar:

```yaml
environment:
  - ROS_DOMAIN_ID=0
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Parámetros ROS2

Editar `ros2_ws/src/planner_map/config/params.yaml`:

```yaml
planner_node:
  ros__parameters:
    update_rate: 10.0
    planning_algorithm: "simple"
```

## 📊 Topics ROS2

- `/map` (nav_msgs/OccupancyGrid): Mapa de ocupación
- `/goal_pose` (geometry_msgs/PoseStamped): Objetivo de navegación
- `/planned_path` (nav_msgs/Path): Ruta planificada
- `/cmd_vel` (geometry_msgs/Twist): Comandos de velocidad

## 🐛 Troubleshooting

### Docker no inicia
```bash
# Verificar Docker
docker --version
docker-compose --version

# Reiniciar servicios
docker-compose down
docker-compose up --build
```

### Puerto 8000 ocupado
```bash
# Cambiar puerto en docker-compose.yml
ports:
  - "8001:8000"  # Usar 8001 en lugar de 8000
```

### Logs de servicios
```bash
# Ver logs de ROS2
docker-compose logs ros2

# Ver logs de Web
docker-compose logs web

# Seguir logs en tiempo real
docker-compose logs -f
```

## 📝 Licencia

Apache-2.0

## 👥 Contribuir

1. Fork el proyecto
2. Crear rama feature (`git checkout -b feature/AmazingFeature`)
3. Commit cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abrir Pull Request

## 📧 Contacto

Maintainer - maintainer@example.com

Project Link: [https://github.com/DahuiLin/planner_map](https://github.com/DahuiLin/planner_map)
