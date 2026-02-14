# Implementación Lanelet2 - Resumen

## Cambios Realizados

Este documento resume todos los cambios implementados para migrar el sistema de OSM/NetworkX a Lanelet2.

### ✅ Archivos Nuevos Creados

1. **`ros2_ws/src/planner_map/planner_map/lanelet2_map_loader.py`**
   - Nuevo cargador de mapas basado en Lanelet2
   - **Lanelet2 lee archivos OSM directamente** usando su parser nativo
   - Características:
     - Carga archivos OSM con formato Lanelet2 usando `lanelet2.io.load()`
     - Construye grafo de enrutamiento con reglas de tráfico
     - Encuentra rutas usando algoritmos de Lanelet2
     - Convierte coordenadas GPS ↔ XY locales
     - Extrae lanelets para visualización

2. **`LANELET2_INTEGRATION.md`**
   - Documentación completa de la integración Lanelet2
   - Incluye guías de uso, troubleshooting, y referencias
   - 300+ líneas de documentación técnica

### ✅ Archivos Modificados

1. **`ros2_ws/src/planner_map/planner_map/planner_node.py`**
   - **CAMBIO CRÍTICO:** Ahora requiere GPS del vehículo (`/fix` topic)
   - Añadida suscripción a `sensor_msgs/NavSatFix`
   - La posición GPS del vehículo es SIEMPRE el punto inicial de la ruta
   - El punto final viene de la interfaz web (seleccionado por el usuario)
   - Usa Lanelet2 para planificación de rutas
   - Publica ruta a `/planned_path` para RViz y web

2. **`ros2_ws/src/planner_map/planner_map/map_server.py`**
   - Migrado de OSM a Lanelet2
   - Publica metadatos Lanelet2 en `/map_metadata`
   - Genera OccupancyGrid desde lanelets

3. **`ros2_ws/src/planner_map/package.xml`**
   - Añadida dependencia: `sensor_msgs` (para NavSatFix)

4. **`ros2_ws/requirements.txt`**
   - **ELIMINADO:** networkx, osmium, pyproj
   - **MANTENIDO:** requests, numpy
   - Lanelet2 se compila desde fuente en Docker

5. **`web_interface/main.py`**
   - Actualizado para manejar metadatos Lanelet2
   - Añadido campo `map_library` en status API
   - Compatible con ambos formatos (Lanelet2 y OSM legado)

6. **`README.md`**
   - Actualizado con información de Lanelet2
   - Instrucciones de uso con GPS
   - Referencias a nueva documentación

### ✅ Archivos Eliminados

- **`osm_map_loader.py`** - Eliminado (Lanelet2 lee OSM directamente con su parser nativo)

### ✅ Archivos Mantenidos (No Modificados)

- `ros2_web_bridge.py` - Sin cambios (funciona igual con Lanelet2)
- Dockerfile.ros - Ya contenía instalación de Lanelet2

## Requisitos del Sistema

### Nuevos Requisitos

1. **Topic GPS Obligatorio:**
   - Topic: `/fix`
   - Tipo: `sensor_msgs/NavSatFix`
   - Propósito: Posición inicial del vehículo para planificación
   - **CRÍTICO:** Sin GPS, no hay planificación de rutas

2. **Formato de Mapa:**
   - Mapas deben estar en formato Lanelet2 (OSM con tags específicos)
   - No son compatibles mapas OSM genéricos sin procesamiento

### Flujo de Planificación de Rutas

```
1. Vehículo publica GPS → /fix (sensor_msgs/NavSatFix)
2. Usuario selecciona destino → Web API → /goal_pose (geometry_msgs/PoseStamped)
3. planner_node recibe ambos
4. Lanelet2 encuentra ruta más corta
5. Ruta publicada → /planned_path (nav_msgs/Path)
6. ros2_web_bridge reenvía a Web API
7. Usuario ve ruta en:
   - Interfaz Web (navegador)
   - RViz (visualización ROS2)
```

## API y Topics

### Topics ROS2 Nuevos/Modificados

- **`/fix`** (sensor_msgs/NavSatFix) - 🆕 NUEVO
  - Posición GPS del vehículo
  - Requerido para planificación

- **`/map_metadata`** (std_msgs/String) - MODIFICADO
  - Ahora contiene metadatos Lanelet2:
    ```json
    {
      "type": "lanelet2",
      "num_lanelets": 150,
      "num_areas": 5,
      "bounds": {...},
      "lanelets": [...]
    }
    ```

### API Web

- **`GET /api/status`** - Añadido campo `map_library: "Lanelet2"`
- **`GET /api/map/osm`** - Compatible con Lanelet2 y OSM legado
- Resto de endpoints sin cambios

## Ventajas de Lanelet2

1. **Mapas a Nivel de Carril:**
   - Información detallada de carriles
   - Elementos regulatorios (semáforos, señales)
   - Límites de velocidad

2. **Reglas de Tráfico:**
   - Enrutamiento inteligente
   - Respeta dirección de vías
   - Considera tipo de vehículo

3. **Mejor Precisión:**
   - Routing graph optimizado
   - Soporte para cambios de carril
   - Coordinación más precisa

## Pruebas Requeridas

**NOTA:** Las pruebas requieren un entorno de ejecución (Docker). No se pueden realizar en este entorno de desarrollo.

### Pruebas Necesarias (cuando se despliegue):

1. **Test de Carga de Mapa:**
   ```bash
   # Verificar que Lanelet2 carga correctamente
   docker compose logs ros2 | grep -i lanelet
   ```

2. **Test de GPS:**
   ```bash
   # Publicar posición GPS de prueba
   ros2 topic pub /fix sensor_msgs/NavSatFix "{...}"
   ros2 topic echo /fix
   ```

3. **Test de Planificación:**
   - Seleccionar destino en web interface
   - Verificar que se calcula ruta
   - Verificar ruta en `/planned_path`

4. **Test de Visualización:**
   - Verificar ruta en interfaz web
   - Verificar ruta en RViz

## Problemas Potenciales y Soluciones

### 1. "Lanelet2 library is not available"
**Solución:** Reconstruir contenedor Docker
```bash
docker compose down
docker compose build --no-cache ros2
docker compose up
```

### 2. "No vehicle GPS position available"
**Solución:** Verificar topic `/fix`
```bash
ros2 topic echo /fix
ros2 topic pub /fix sensor_msgs/NavSatFix "{latitude: 48.98403, longitude: 8.39014, altitude: 115.0, status: {status: 0, service: 1}}" --once
```

### 3. "No path found using Lanelet2 routing"
**Causas posibles:**
- Mapa no está en formato Lanelet2
- Puntos inicio/fin demasiado lejos de lanelets (>50m)
- Lanelets no conectados en el mapa

**Solución:** Verificar formato de mapa, usar JOSM con plugin Lanelet2

### 4. Lectura de archivos OSM
**Importante:** Lanelet2 lee archivos OSM directamente con su parser nativo (`lanelet2.io.load()`).
- No se requiere parser OSM separado
- El antiguo `osm_map_loader.py` ha sido eliminado
- Mapas deben estar en formato Lanelet2 (OSM con tags de carriles)
- Mapas OSM genéricos requieren conversión

**Solución para mapas OSM genéricos:** Convertir a formato Lanelet2
- Usar herramientas de conversión Lanelet2
- Editar con JOSM + plugin Lanelet2
- Ver documentación oficial de Lanelet2

## Compatibilidad hacia Atrás

- El archivo `osm_map_loader.py` **ha sido eliminado**
- Lanelet2 lee archivos OSM directamente - no se necesita parser separado
- Sistema NO es compatible con mapas OSM genéricos
- Mapas deben convertirse a formato Lanelet2

## Próximos Pasos Sugeridos

1. **Probar con Mapa Real:**
   - Obtener o crear mapa Lanelet2 del área de prueba
   - Cargar y verificar funcionalidad

2. **Integrar GPS Real:**
   - Conectar módulo GPS físico
   - Publicar a `/fix` topic

3. **Optimizar Rendimiento:**
   - Medir tiempo de carga de mapas grandes
   - Optimizar búsqueda de lanelets cercanos

4. **Mejorar Visualización:**
   - Mostrar lanelets individuales en web
   - Mostrar elementos regulatorios
   - Colores por tipo de carril

5. **Funciones Avanzadas:**
   - Diferentes funciones de costo (tiempo, distancia, confort)
   - Soporte para múltiples tipos de vehículos
   - Enrutamiento dinámico con tráfico

## Referencias

- [Lanelet2 GitHub](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [Lanelet2 Paper](https://arxiv.org/abs/1809.10728)
- [LANELET2_INTEGRATION.md](LANELET2_INTEGRATION.md) - Documentación técnica completa
- [ROS2 NavSatFix](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)

## Contacto y Soporte

Para problemas o preguntas:
1. Revisar LANELET2_INTEGRATION.md
2. Revisar troubleshooting en esta guía
3. Consultar documentación oficial de Lanelet2
4. Abrir issue en el repositorio del proyecto

---

**Fecha de Implementación:** 2026-02-13
**Versión:** 1.0
**Estado:** ✅ Implementación Completa - Pendiente de Pruebas en Runtime
