# Planner Map - ROS2 + FastAPI + Docker + Lanelet2

![CI - Build and Test](https://github.com/DahuiLin/planner_map/workflows/CI%20-%20Build%20and%20Test/badge.svg)
![Deploy to Production](https://github.com/DahuiLin/planner_map/workflows/Deploy%20to%20Production/badge.svg)

Sistema de planificación y mapeo integrado con ROS2, interfaz web FastAPI, soporte para mapas Lanelet2 y despliegue automatizado con Docker Compose.

## 📋 Descripción

Este proyecto combina:
- **ROS2 Humble**: Sistema de planificación de rutas y servidor de mapas
- **FastAPI**: Interfaz web moderna para visualización y control
- **Docker Compose**: Despliegue automatizado de todos los servicios
- **ROS2-Web Bridge**: Comunicación bidireccional entre ROS2 y la interfaz web
- **🗺️ Lanelet2**: Biblioteca avanzada para mapas a nivel de carril y enrutamiento
- **🚗 Car Navigation**: Navegación con reglas de tráfico y enrutamiento inteligente
- **📡 GPS Integration**: Posición del vehículo desde topic `/fix` (NavSatFix)

## 🔗 ¿Cómo está Conectado?

La interfaz web y ROS2 se comunican a través de un **nodo puente** (`ros2_web_bridge`) que coordina ambos sistemas:

- 📤 **Web → ROS2**: Los objetivos del usuario se envían a través de la API REST, el bridge los detecta y publica en topics ROS2
- 📥 **ROS2 → Web**: Los mapas y rutas se publican en topics, el bridge los captura y envía a la API web para visualización

**📖 Documentación detallada:**
- **[CONEXION.md](CONEXION.md)** - Guía completa en español sobre cómo funciona la comunicación
- **[CONNECTION_DIAGRAM.md](CONNECTION_DIAGRAM.md)** - Diagramas visuales del flujo de datos
- **[LANELET2_INTEGRATION.md](LANELET2_INTEGRATION.md)** - 🆕 Guía completa de integración con Lanelet2
- **[GUIA_MAPAS_PERSONALIZADOS.md](GUIA_MAPAS_PERSONALIZADOS.md)** - 🆕 **Guía para cargar mapas personalizados**
- **[OSM_SUPPORT.md](OSM_SUPPORT.md)** - Guía de soporte OSM (legado)
- **[CI_CD.md](CI_CD.md)** - 🆕 **Documentación de CI/CD y deployment**

## 🏗️ Estructura del Proyecto

```
planner_map/
├── ros2_ws/                    # ROS2 Workspace
│   └── src/
│       └── planner_map/        # Paquete ROS2
│           ├── planner_map/    # Código Python
│           │   ├── planner_node.py      # Nodo de planificación
│           │   ├── map_server.py        # Servidor de mapas
│           │   └── ros2_web_bridge.py   # Puente ROS2 ↔ Web
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
├── CONEXION.md                # Guía de conexión (Español)
├── CONNECTION_DIAGRAM.md      # Diagramas de conexión
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
  
- **ROS2 Nodes** (se inician automáticamente):
  - `planner_node`: Nodo de planificación de rutas
  - `map_server`: Servidor de mapas
  - `ros2_web_bridge`: Puente de comunicación ROS2 ↔ Web

### Verificar la Conexión

```bash
# Ver nodos ROS2 activos (debe incluir ros2_web_bridge)
docker-compose exec ros2 ros2 node list

# Ver topics (debe incluir /map, /goal_pose, /planned_path)
docker-compose exec ros2 ros2 topic list

# Monitorear comunicación del bridge
docker-compose logs -f ros2 | grep bridge
```

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
- ✅ **🆕 Soporte para Lanelet2 - mapas a nivel de carril**
- ✅ **🆕 Integración GPS vía topic /fix (sensor_msgs/NavSatFix)**
- ✅ **🆕 Enrutamiento con reglas de tráfico (Lanelet2 routing)**
- ✅ **🆕 Posición inicial del vehículo desde GPS**
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

## 🗺️ Usando Lanelet2

### Inicio Rápido con Lanelet2

1. **Preparar posición GPS del vehículo**:
   ```bash
   # Publicar posición GPS de prueba
   ros2 topic pub /fix sensor_msgs/NavSatFix "{
     latitude: 48.98403,
     longitude: 8.39014,
     altitude: 115.0,
     status: {status: 0, service: 1}
   }" --once
   ```

2. **Usar un mapa Lanelet2**:
   - Los mapas Lanelet2 son archivos OSM con información de carriles
   - Coloca tu mapa en `config/your_map.osm`
   - El mapa debe tener formato Lanelet2 (con tags específicos)
   - **📖 Ver [GUIA_MAPAS_PERSONALIZADOS.md](GUIA_MAPAS_PERSONALIZADOS.md) para crear tu propio mapa**

3. **Configurar el mapa**:
   ```bash
   # Edita docker-compose.yml para especificar tu archivo
   ros2 launch planner_map planner_map.launch.py \
     osm_file:=/workspace/config/your_map.osm
   ```

4. **Seleccionar destino**:
   - Usa la interfaz web para seleccionar el punto final
   - El sistema calculará la ruta desde la posición GPS del vehículo

### Características Lanelet2

- 🗺️ Mapas a nivel de carril con información detallada
- 🚗 Reglas de tráfico integradas (alemanas por defecto)
- 🛣️ Soporte para elementos regulatorios (semáforos, límites de velocidad)
- 📏 Enrutamiento inteligente considerando reglas de tráfico
- 🌐 Conversión automática GPS ↔ coordenadas locales
- 📊 Visualización de red de carriles en interfaz web

**Ver [LANELET2_INTEGRATION.md](LANELET2_INTEGRATION.md) para documentación completa**

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
- `/fix` (sensor_msgs/NavSatFix): **🆕 Posición GPS del vehículo (requerido para planificación)**
- `/map_metadata` (std_msgs/String): Metadatos del mapa Lanelet2

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

## 🚀 CI/CD

Este proyecto incluye flujos de trabajo automatizados de CI/CD usando GitHub Actions.

### Integración Continua

Cada push y pull request ejecuta automáticamente:
- ✅ Build de imágenes Docker (ROS2 y Web)
- ✅ Validación de docker-compose.yml
- ✅ Health checks de servicios
- ✅ Tests de API endpoints
- ✅ Verificación de nodos ROS2
- ✅ Tests de integración end-to-end
- ✅ Análisis de código con linters

### Deployment

Deployment automático mediante:
- 🏷️ **Tags de versión**: `git tag v1.0.0 && git push origin v1.0.0`
- 🚀 **Manual dispatch**: Desde la pestaña Actions en GitHub

### Tests Locales

```bash
# Ejecutar tests de integración
python3 test_integration.py

# Verificar configuración
docker compose config

# Ver estado de health checks
docker compose ps
```

**📖 Ver [CI_CD.md](CI_CD.md) para documentación completa de CI/CD**

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
