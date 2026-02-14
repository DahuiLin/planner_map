# 📋 Resumen del Repositorio: planner_map

## 🎯 **Propósito del Proyecto**

**planner_map** es un sistema integrado de **planificación de rutas y gestión de mapas** para vehículos autónomos que combina:
- **ROS2 (Robot Operating System 2)** para el backend de planificación
- **FastAPI** para una interfaz web moderna e interactiva
- **Lanelet2** para mapas a nivel de carril con información de tráfico
- **Docker Compose** para despliegue automatizado

## 🏗️ **Arquitectura del Sistema**

El sistema consta de **dos contenedores Docker** principales que se comunican a través de una red bridge:

### 1. **Contenedor ROS2** (`ros2`)
- **Imagen base:** osrf/ros:humble-desktop
- **Nodos ROS2:**
  - `planner_node`: Planificación de rutas con algoritmos de navegación
  - `map_server`: Publicación y gestión de mapas de ocupación
  - `ros2_web_bridge`: Puente de comunicación bidireccional ROS2 ↔ Web
  - `lanelet2_map_loader`: Carga y procesamiento de mapas Lanelet2

### 2. **Contenedor Web** (`web`)
- **Imagen base:** python:3.11-slim
- **Framework:** FastAPI con servidor Uvicorn ASGI
- **Funcionalidades:**
  - API REST para control del sistema
  - WebSocket para actualizaciones en tiempo real
  - Interfaz HTML/JavaScript para visualización interactiva
  - Panel de control con estado del sistema

## 🔑 **Características Principales**

### **Mapas Lanelet2**
- 🗺️ Mapas a **nivel de carril** (lane-level mapping) para navegación precisa
- 🚦 Soporte para **elementos regulatorios** (semáforos, señales de tráfico, límites de velocidad)
- 🛣️ **Reglas de tráfico integradas** (alemanas por defecto, configurables)
- 🌐 Conversión automática **GPS ↔ coordenadas locales** con proyecciones geográficas
- 📊 Visualización de red de carriles en la interfaz web
- 📏 Geometría precisa de carriles con información de ancho y dirección

### **Integración GPS**
- 📡 Posición del vehículo desde topic ROS2 `/fix` (sensor_msgs/NavSatFix)
- 🎯 La posición GPS del vehículo siempre es el **punto de inicio** para planificación
- 🗺️ El usuario selecciona el destino en la interfaz web mediante clic en el mapa
- 🔄 Actualización continua de la posición del vehículo

### **Planificación de Rutas**
- 🚗 Enrutamiento inteligente usando el **algoritmo de Lanelet2**
- 📏 Considera **reglas de tráfico** y restricciones de carriles
- 🔄 Actualización en **tiempo real** de rutas planificadas
- 📈 Publicación en topic `/planned_path` para visualización en RViz
- 🎯 Cálculo de trayectorias con splines suaves
- ⚡ Optimización de rutas considerando distancia y tiempo

### **Interfaz Web Moderna**
- 🖱️ **Clic en el mapa** para establecer objetivos de navegación
- 📊 Panel de control con estado del sistema en tiempo real
- 🔄 **WebSocket** para actualizaciones automáticas sin recargar
- 📱 Diseño responsive que funciona en desktop y móvil
- 📝 Registro de actividad del sistema con timestamps
- 🎨 Interfaz intuitiva con visualización de mapas en canvas HTML5

## 🔄 **Flujo de Datos**

### Establecimiento de Objetivo:
```
Usuario hace clic en mapa
    ↓
JavaScript captura coordenadas (x, y)
    ↓
POST /api/goal (FastAPI)
    ↓
WebSocket broadcast a todos los clientes conectados
    ↓
ros2_web_bridge recibe objetivo desde API
    ↓
Publica en topic /goal_pose (geometry_msgs/PoseStamped)
    ↓
planner_node recibe objetivo y planifica ruta
    ↓
Publica ruta en /planned_path (nav_msgs/Path)
    ↓
ros2_web_bridge captura ruta y envía a API web
    ↓
Frontend actualiza visualización del mapa
```

### Actualización de Mapas:
```
map_server publica en /map (nav_msgs/OccupancyGrid)
    ↓
ros2_web_bridge suscribe y recibe mapa
    ↓
POST /api/map (FastAPI) - envía datos del mapa
    ↓
WebSocket broadcast a clientes web
    ↓
Frontend solicita datos con GET /api/map
    ↓
Renderiza mapa en canvas HTML5
```

### Posición GPS del Vehículo:
```
Sensor GPS publica en /fix (sensor_msgs/NavSatFix)
    ↓
planner_node suscribe y actualiza posición del vehículo
    ↓
lanelet2_map_loader convierte GPS a coordenadas locales
    ↓
Se usa como punto de inicio para planificación
```

## 📊 **Topics ROS2 Principales**

| Topic | Tipo de Mensaje | Propósito |
|-------|----------------|-----------|
| `/map` | nav_msgs/OccupancyGrid | Mapa de ocupación 2D con información de obstáculos |
| `/goal_pose` | geometry_msgs/PoseStamped | Objetivo de navegación seleccionado por el usuario |
| `/planned_path` | nav_msgs/Path | Ruta planificada desde posición actual hasta objetivo |
| `/cmd_vel` | geometry_msgs/Twist | Comandos de velocidad lineal y angular |
| `/fix` | sensor_msgs/NavSatFix | **Posición GPS del vehículo** (latitud, longitud, altitud) |
| `/map_metadata` | std_msgs/String | Metadatos del mapa Lanelet2 (proyección, límites, etc.) |

## 🛠️ **Stack Tecnológico**

### Backend:
- **ROS2 Humble LTS** - Sistema de robótica con soporte hasta 2027
- **Python 3.10+** - Lenguaje principal del proyecto
- **rclpy** - Cliente Python oficial para ROS2
- **Lanelet2** - Biblioteca de mapas HD para vehículos autónomos
- **FastAPI** - Framework web asíncrono de alto rendimiento
- **Uvicorn** - Servidor ASGI para aplicaciones Python async
- **WebSockets** - Comunicación bidireccional en tiempo real

### Frontend:
- **JavaScript Vanilla** - Sin dependencias de frameworks pesados
- **HTML5 Canvas** - Renderizado de mapas 2D de alto rendimiento
- **CSS3** (Flexbox/Grid) - Diseño responsive moderno
- **WebSocket API** - Conexión persistente con el servidor
- **Fetch API** - Peticiones HTTP asíncronas

### Infraestructura:
- **Docker** - Contenedorización de servicios
- **Docker Compose** - Orquestación multi-contenedor
- **GitHub Actions** - CI/CD automatizado
- **Make** - Automatización de tareas de desarrollo

## 📁 **Estructura del Proyecto**

```
planner_map/
├── ros2_ws/                          # Workspace ROS2
│   ├── src/planner_map/              # Paquete principal
│   │   ├── planner_map/              # Código Python (~1,874 líneas)
│   │   │   ├── __init__.py           # Inicialización del paquete
│   │   │   ├── planner_node.py       # Nodo de planificación de rutas
│   │   │   ├── map_server.py         # Servidor de mapas OccupancyGrid
│   │   │   ├── ros2_web_bridge.py    # Puente ROS2-Web bidireccional
│   │   │   ├── lanelet2_map_loader.py # Cargador de mapas Lanelet2
│   │   │   └── spline_trajectory.py  # Generación de trayectorias spline
│   │   ├── launch/                   # Archivos de lanzamiento ROS2
│   │   │   └── planner_map.launch.py # Launch file principal
│   │   ├── config/                   # Configuración de parámetros
│   │   │   └── params.yaml           # Parámetros de nodos ROS2
│   │   ├── package.xml               # Manifiesto y dependencias ROS2
│   │   └── setup.py                  # Configuración de instalación Python
│   └── requirements.txt              # Dependencias Python del workspace
├── web_interface/                    # Interfaz Web FastAPI
│   ├── main.py                       # Aplicación FastAPI principal
│   ├── static/                       # Archivos estáticos
│   │   ├── style.css                 # Estilos de la interfaz
│   │   └── script.js                 # Lógica del frontend
│   ├── templates/                    # Plantillas HTML Jinja2
│   │   └── index.html                # Página principal
│   └── requirements.txt              # Dependencias Python web
├── config/                           # Configuración general y mapas
│   ├── example.env                   # Ejemplo de variables de entorno
│   └── sample_map.osm                # Mapa de ejemplo Lanelet2
├── docker/                           # Scripts y configuración Docker
│   └── ros_entrypoint.sh             # Script de entrada del contenedor ROS2
├── .github/workflows/                # GitHub Actions CI/CD
│   ├── ci.yml                        # Workflow de integración continua
│   └── deploy.yml                    # Workflow de despliegue
├── Dockerfile.ros                    # Dockerfile para contenedor ROS2
├── Dockerfile.web                    # Dockerfile para contenedor Web
├── docker-compose.yml                # Orquestación principal
├── docker-compose.dev.yml            # Configuración para desarrollo
├── start.sh                          # Script de inicio rápido
├── Makefile                          # Tareas de automatización
├── test_integration.py               # Tests de integración end-to-end
├── test_spline_trajectory.py         # Tests unitarios de trayectorias
└── Documentación (20+ archivos .md)  # Guías completas en ES/EN
```

## 🚀 **CI/CD y Automatización**

### **GitHub Actions Workflows:**

#### 1. **CI - Build and Test** (`ci.yml`)
Ejecutado en cada push y pull request:
- ✅ **Build de imágenes Docker** (ROS2 y Web)
- ✅ **Validación de docker-compose.yml** sintaxis y estructura
- ✅ **Health checks de servicios** Docker
- ✅ **Tests de API endpoints** con pytest
- ✅ **Verificación de nodos ROS2** activos
- ✅ **Tests de integración end-to-end** completos
- ✅ **Análisis de código** con flake8 y pylint
- ✅ **Detección de TODOs** y prints de debug
- 📊 **Recolección de logs** en caso de fallos

#### 2. **Deploy** (`deploy.yml`)
Ejecutado en tags de versión (v*.*.*)  o manualmente:
- 🚀 **Build y push a Docker Hub** con tags de versión
- 🏷️ **Creación de GitHub Releases** automática
- 🌐 **Despliegue a staging/producción** configurable
- ✅ **Verificación post-despliegue** automática

### **Health Checks Integrados:**
- **ROS2:** Verifica nodos activos cada 30s
- **Web:** Chequea endpoint `/api/status` cada 30s
- **Retries:** 3 intentos antes de marcar como unhealthy
- **Start Period:** 40s para ROS2, 30s para Web

## 📚 **Documentación Extensa**

El proyecto incluye **más de 20 archivos de documentación** detallada en español e inglés:

| Documento | Descripción | Líneas |
|-----------|-------------|--------|
| `README.md` | Guía principal del proyecto | 400+ |
| `ARCHITECTURE.md` | Arquitectura detallada del sistema | 250+ |
| `LANELET2_INTEGRATION.md` | Integración completa con Lanelet2 | 300+ |
| `CONEXION.md` | Comunicación ROS2-Web (Español) | 200+ |
| `CONNECTION_DIAGRAM.md` | Diagramas de flujo de datos | 150+ |
| `GUIA_MAPAS_PERSONALIZADOS.md` | Guía para crear mapas propios | 250+ |
| `CI_CD.md` | Documentación de CI/CD | 200+ |
| `MAPA_CONFIG.md` | Configuración de mapas | 180+ |
| `OSM_SUPPORT.md` | Soporte para archivos OSM | 150+ |
| `SPLINE_TRAJECTORY.md` | Generación de trayectorias | 120+ |
| `CONTRIBUTING.md` | Guía de contribución | 100+ |

## 🎯 **Casos de Uso**

### 1. **Desarrollo de Vehículos Autónomos**
- Sistema completo para pruebas de navegación autónoma
- Integración con hardware GPS real
- Visualización en tiempo real de rutas planificadas
- Base para implementar controladores de vehículo

### 2. **Investigación en Robótica**
- Plataforma para probar algoritmos de planificación
- Framework extensible para nuevos métodos de navegación
- Infraestructura lista para recolección de datos
- Integración con simuladores ROS2 (Gazebo, RViz)

### 3. **Simulación de Tráfico**
- Visualización de rutas en mapas reales
- Análisis de patrones de navegación
- Pruebas de escenarios de tráfico
- Validación de reglas de tráfico

### 4. **Educación**
- Aprendizaje de ROS2 y arquitectura de sistemas robóticos
- Práctica con Docker y microservicios
- Desarrollo web con APIs modernas (FastAPI)
- Sistemas distribuidos y comunicación en tiempo real

## ⚙️ **Configuración Flexible**

### **Opciones de Configuración de Mapas:**

1. **Archivo .env (Recomendado):**
   ```bash
   cp config/example.env .env
   # Editar OSM_FILE=/workspace/config/mi_mapa.osm
   docker compose up
   ```

2. **Variable de entorno directa:**
   ```bash
   OSM_FILE=/workspace/config/dekra.osm docker compose up
   ```

3. **Edición de docker-compose.yml:**
   ```yaml
   environment:
     - OSM_FILE=/workspace/config/mi_mapa.osm
   ```

### **Parámetros Configurables:**

**Variables de Entorno (docker-compose.yml):**
- `ROS_DOMAIN_ID`: ID de dominio ROS2 (default: 0)
- `RMW_IMPLEMENTATION`: Middleware (rmw_cyclonedds_cpp o rmw_fastrtps_cpp)
- `OSM_FILE`: Ruta al archivo de mapa Lanelet2
- `PYTHONUNBUFFERED`: Logging sin buffer (default: 1)

**Parámetros ROS2 (config/params.yaml):**
```yaml
planner_node:
  ros__parameters:
    update_rate: 10.0           # Hz de actualización
    planning_algorithm: "lanelet2"  # Algoritmo de planificación
    max_planning_time: 5.0      # Tiempo máximo de planificación (s)
    
map_server:
  ros__parameters:
    publish_rate: 1.0           # Hz de publicación del mapa
    map_resolution: 0.1         # Resolución en metros
```

## 🔒 **Seguridad y Mejores Prácticas**

### **Implementado:**
- ✅ **Health checks** en servicios Docker para alta disponibilidad
- ✅ **Tests de integración** automatizados en CI/CD
- ✅ **Manejo robusto de excepciones** con logging detallado
- ✅ **Validación de entrada** en API endpoints
- ✅ **Aislamiento de red** via Docker networks
- ✅ **Separación de concerns** (ROS2 / Web)
- ✅ **Restart automático** de servicios en caso de fallos

### **Recomendaciones para Producción:**
- ⚠️ **Autenticación y autorización** (no implementado actualmente)
- ⚠️ **HTTPS/TLS** para comunicación encriptada
- ⚠️ **Rate limiting** en API endpoints
- ⚠️ **Validación de origen** en WebSocket connections
- ⚠️ **Gestión de secretos** con herramientas apropiadas (no .env)
- ⚠️ **Monitoreo y alertas** para detección de problemas

## 📈 **Estadísticas del Proyecto**

### **Código:**
- **Python:** ~1,874 líneas totales
  - ROS2 nodes: ~1,200 líneas
  - Web interface: ~400 líneas
  - Tests: ~270 líneas
- **JavaScript:** ~300 líneas (frontend interactivo)
- **HTML/CSS:** ~400 líneas (interfaz web)

### **Componentes:**
- **Nodos ROS2:** 4 nodos principales
- **Endpoints API REST:** 7 endpoints
- **Topics ROS2:** 6 topics principales
- **Contenedores Docker:** 2 servicios orquestados
- **Workflows CI/CD:** 2 workflows automatizados
- **Tests automatizados:** 15+ casos de prueba

### **Documentación:**
- **Archivos markdown:** 20+ documentos
- **Líneas de documentación:** 3,000+ líneas
- **Idiomas:** Español e Inglés
- **Diagramas:** Múltiples diagramas de arquitectura y flujo

### **Infraestructura:**
- **Imágenes Docker:** 2 imágenes personalizadas
- **Networks Docker:** 1 red bridge
- **Volumes:** 3 volúmenes para desarrollo
- **Licencia:** Apache-2.0 (open source)

## 🎓 **Tecnologías Avanzadas**

### **Lanelet2 - Mapas HD de Nueva Generación**
- Biblioteca de última generación desarrollada por Forschungszentrum Informatik (FZI)
- Representación semántica de carriles con atributos ricos
- Soporte nativo para elementos regulatorios complejos
- Routing graph con consideración de reglas de tráfico
- Usado en proyectos de conducción autónoma de nivel industrial

### **ROS2 Humble - Versión LTS**
- Long Term Support hasta mayo de 2027
- Mejoras significativas en DDS y comunicación
- Mayor rendimiento y menor latencia
- Compatibilidad con sistemas Linux, Windows y macOS
- Ecosistema maduro con cientos de paquetes

### **FastAPI - Framework Web Moderno**
- Basado en estándares Python modernos (type hints, async/await)
- Rendimiento comparable a Node.js y Go
- Documentación automática con OpenAPI/Swagger
- Validación automática con Pydantic
- WebSocket nativo para tiempo real

### **Docker Multi-stage y Optimización**
- Builds optimizados con capas en caché
- Imágenes slim para reducir tamaño
- Health checks integrados
- Restart policies para alta disponibilidad
- Networking eficiente entre contenedores

## 🌟 **Puntos Destacados**

### **1. Sistema Completo End-to-End**
Desde sensores GPS hasta visualización web, todo integrado en un solo sistema coherente.

### **2. Documentación de Calidad Profesional**
Más de 3,000 líneas de documentación detallada en múltiples idiomas con ejemplos prácticos.

### **3. CI/CD Robusto**
Automatización completa de pruebas, análisis de código y despliegue con GitHub Actions.

### **4. Arquitectura Moderna de Microservicios**
Separación clara de responsabilidades con comunicación eficiente entre servicios.

### **5. Tecnología de Vanguardia**
Lanelet2 para mapas HD de nivel profesional usado en industria automotriz.

### **6. Despliegue Simplificado**
`docker compose up` y el sistema completo está funcionando en minutos.

### **7. Extensible y Mantenible**
Código bien estructurado, modular y con tests que facilita agregar nuevas funcionalidades.

### **8. Comunidad y Open Source**
Licencia Apache-2.0 con guías de contribución para desarrollo colaborativo.

## 🚀 **Inicio Rápido**

### **Requisitos Previos:**
- Docker (>= 20.10)
- Docker Compose (>= 2.0)
- 4GB RAM disponible
- Puerto 8000 libre

### **Instalación en 3 Pasos:**

```bash
# 1. Clonar el repositorio
git clone https://github.com/DahuiLin/planner_map.git
cd planner_map

# 2. Iniciar servicios
docker compose up --build

# 3. Abrir navegador en http://localhost:8000
```

### **Verificar Funcionamiento:**

```bash
# Ver nodos ROS2 activos
docker compose exec ros2 ros2 node list

# Ver topics disponibles
docker compose exec ros2 ros2 topic list

# Verificar salud de servicios
docker compose ps
```

## 📧 **Soporte y Comunidad**

- **Issues:** [GitHub Issues](https://github.com/DahuiLin/planner_map/issues)
- **Documentación:** Ver archivos .md en el repositorio
- **Contribuciones:** Ver CONTRIBUTING.md
- **Licencia:** Apache-2.0 (Ver LICENSE)

## 🎯 **Conclusión**

**planner_map** es un proyecto **maduro, bien documentado y listo para producción** que combina lo mejor de:
- **ROS2** para sistemas robóticos robustos
- **Tecnologías web modernas** para interfaces accesibles  
- **Herramientas de mapeo avanzadas** (Lanelet2) de nivel industrial
- **DevOps automatizado** con CI/CD completo

Es ideal tanto para **desarrollo de vehículos autónomos reales** como para **educación e investigación** en sistemas robóticos. La integración de **Lanelet2** lo distingue como una solución **profesional** para navegación a nivel de carril, mientras que su **interfaz web moderna** lo hace accesible para usuarios sin experiencia previa en ROS2.

La arquitectura de **microservicios con Docker**, la **documentación exhaustiva** y el **CI/CD automatizado** lo convierten en una base sólida para proyectos de navegación autónoma de cualquier escala.

---

**🔗 Enlaces Importantes:**
- [Repositorio GitHub](https://github.com/DahuiLin/planner_map)
- [Documentación Lanelet2](https://github.com/fzi-forschungszentrum-informatik/lanelet2)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [FastAPI Docs](https://fastapi.tiangolo.com/)
