# Guía para Cargar Mapas Personalizados con Lanelet2

## 📋 Introducción

Esta guía explica paso a paso cómo cargar un mapa personalizado en el sistema de planificación, qué datos se necesitan, y cómo asegurar que todo funcione correctamente.

## 🗺️ ¿Qué es un Mapa Lanelet2?

Lanelet2 es un formato de mapa diseñado para vehículos autónomos que incluye:
- **Carriles individuales** (lanelets) con límites izquierdo y derecho
- **Reglas de tráfico** (sentido único, velocidad máxima, etc.)
- **Elementos regulatorios** (semáforos, señales de tráfico, límites de velocidad)
- **Áreas** (zonas de estacionamiento, intersecciones)
- **Información topológica** (conexiones entre carriles)

## 📦 Requisitos Previos

### 1. Datos Necesarios

Para crear un mapa Lanelet2 personalizado necesitas:

#### Datos Geográficos
- **Coordenadas GPS** del área a mapear (latitud, longitud)
- **Límites del área** (bounding box)
- **Imágenes satelitales** o aéreas del área (opcional pero muy útil)

#### Información de la Infraestructura
- **Geometría de los carriles**:
  - Líneas de borde izquierdo y derecho de cada carril
  - Ancho de carriles
  - Conexiones entre carriles

- **Reglas de tráfico**:
  - Sentido de circulación (one-way vs bidireccional)
  - Velocidades máximas
  - Prioridades en intersecciones

- **Elementos regulatorios** (opcional):
  - Posición de semáforos
  - Señales de tráfico
  - Zonas de límite de velocidad

### 2. Herramientas Necesarias

1. **JOSM (Java OpenStreetMap Editor)** - Editor OSM con plugin Lanelet2
2. **Plugin Lanelet2 para JOSM** - Extensión específica para mapas Lanelet2
3. **Lanelet2 Python** - Para validar el mapa

## 🛠️ Métodos para Crear un Mapa Personalizado

### Método 1: Crear Desde Cero con JOSM (Recomendado)

#### Paso 1: Instalar JOSM y Plugin Lanelet2

```bash
# Descargar JOSM
wget https://josm.openstreetmap.de/josm-tested.jar

# Ejecutar JOSM
java -jar josm-tested.jar

# Instalar plugin Lanelet2:
# 1. Preferences → Plugins
# 2. Buscar "Lanelet2"
# 3. Instalar y reiniciar
```

#### Paso 2: Crear Nuevo Mapa

1. **Abrir JOSM** y crear un nuevo layer
2. **Cargar imagen de fondo**:
   - Imagery → Choose imagery → Bing/Google Satellite
   - O usar imagen local: Imagery → Rectified Image

3. **Definir el área**:
   - Download → Download from API
   - Seleccionar área deseada (máximo ~0.5 km²)

#### Paso 3: Dibujar Lanelets

1. **Crear líneas de borde** (linestrings):
   ```
   - Seleccionar Draw tool (A)
   - Dibujar línea izquierda del carril
   - Dibujar línea derecha del carril
   - Asignar tags:
     * type=line_thin
     * subtype=solid (o dashed)
   ```

2. **Crear lanelet** (relación):
   ```
   - Seleccionar ambas líneas de borde
   - Create relation (Ctrl+Shift+R)
   - Tags del lanelet:
     * type=lanelet
     * subtype=road (o highway, parking, etc.)
     * location=urban (o rural)
     * one_way=yes (o no)
     * speed_limit=50 (km/h, opcional)
   - Members:
     * Línea izquierda: role=left
     * Línea derecha: role=right
   ```

3. **Conectar lanelets**:
   - Los nodos finales de un carril deben coincidir con nodos iniciales del siguiente
   - Lanelet2 usa estos nodos compartidos para construir el grafo de enrutamiento

#### Paso 4: Añadir Elementos Regulatorios (Opcional)

**Semáforo:**
```xml
<relation>
  <tag k="type" v="regulatory_element"/>
  <tag k="subtype" v="traffic_light"/>
  <member type="node" ref="node_id" role="ref_line"/>
  <member type="way" ref="lanelet_id" role="refers"/>
</relation>
```

**Límite de velocidad:**
```xml
<relation>
  <tag k="type" v="regulatory_element"/>
  <tag k="subtype" v="speed_limit"/>
  <tag k="sign_type" v="274"/>
  <tag k="speed_limit" v="30"/>
  <member type="way" ref="lanelet_id" role="refers"/>
</relation>
```

#### Paso 5: Validar el Mapa

En JOSM:
1. Validation → Validate
2. Revisar errores y warnings
3. Corregir problemas encontrados

#### Paso 6: Exportar el Mapa

```
File → Export → Save as OSM .osm
Guardar en: config/mi_mapa_personalizado.osm
```

### Método 2: Convertir Mapa OSM Existente

Si tienes un mapa OSM estándar (solo carreteras), necesitas convertirlo:

```python
#!/usr/bin/env python3
"""
Script básico para convertir OSM a Lanelet2
NOTA: Requiere procesamiento manual posterior en JOSM
"""
import xml.etree.ElementTree as ET

def osm_to_lanelet2_basic(input_osm, output_osm):
    """Conversión básica - genera estructura inicial"""
    tree = ET.parse(input_osm)
    root = tree.getroot()

    # Procesar ways (carreteras)
    for way in root.findall('way'):
        tags = {tag.get('k'): tag.get('v') for tag in way.findall('tag')}

        if 'highway' in tags:
            # Convertir a lanelet básico
            way.set('id', f"lanelet_{way.get('id')}")

            # Añadir tags Lanelet2
            ET.SubElement(way, 'tag', k='type', v='lanelet')
            ET.SubElement(way, 'tag', k='subtype', v='road')
            ET.SubElement(way, 'tag', k='location', v='urban')

    tree.write(output_osm)
    print(f"Mapa convertido guardado en: {output_osm}")
    print("IMPORTANTE: Abrir en JOSM y ajustar manualmente")

# Uso
osm_to_lanelet2_basic('input.osm', 'output_lanelet2.osm')
```

**⚠️ IMPORTANTE:** La conversión automática solo genera una estructura base. Debes:
1. Abrir el archivo en JOSM con plugin Lanelet2
2. Ajustar geometría de carriles
3. Añadir líneas de borde
4. Corregir conexiones
5. Validar y exportar

### Método 3: Usar Mapas de Ejemplo

Para pruebas rápidas, puedes usar mapas de ejemplo:

```bash
# Descargar mapa de ejemplo de Lanelet2
cd config/
wget https://github.com/fzi-forschungszentrum-informatik/Lanelet2/raw/master/lanelet2_maps/res/mapping_example.osm

# Renombrar
mv mapping_example.osm mi_mapa_ejemplo.osm
```

## 🔧 Configurar el Sistema para Usar el Mapa

### 1. Colocar el Archivo del Mapa

```bash
# Copiar mapa a la carpeta config
cp mi_mapa_personalizado.osm /home/runner/work/planner_map/planner_map/config/
```

### 2. Configurar Docker Compose

Editar `docker-compose.yml`:

```yaml
services:
  ros2:
    # ... otras configuraciones ...
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /tmp/lanelet2_ws/install/setup.bash &&
        source /workspace/ros2_ws/install/setup.bash &&
        ros2 launch planner_map planner_map.launch.py
        osm_file:=/workspace/config/mi_mapa_personalizado.osm
      "
```

### 3. Configurar Launch File (Alternativa)

Editar `ros2_ws/src/planner_map/launch/planner_map.launch.py`:

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planner_map',
            executable='map_server',
            parameters=[{
                'osm_file': '/workspace/config/mi_mapa_personalizado.osm',
                'publish_rate': 1.0
            }]
        ),
        # ... otros nodos ...
    ])
```

## ✅ Verificar que el Mapa Funciona

### 1. Verificar Carga del Mapa

```bash
# Iniciar sistema
docker-compose up

# Ver logs
docker-compose logs ros2 | grep -i lanelet

# Deberías ver:
# "Loading Lanelet2 map from: /workspace/config/mi_mapa_personalizado.osm"
# "Loaded X lanelets and Y areas"
```

### 2. Verificar Metadatos

```bash
# Verificar topic de metadatos
ros2 topic echo /map_metadata --once

# Salida esperada:
# data: '{"type": "lanelet2", "num_lanelets": X, "bounds": {...}}'
```

### 3. Verificar en Web Interface

```bash
# Abrir navegador
firefox http://localhost:8000

# Verificar:
# 1. Status muestra "map_library: Lanelet2"
# 2. Mapa se visualiza correctamente
# 3. Carriles aparecen dibujados
```

### 4. Probar Enrutamiento

```bash
# Publicar posición GPS de prueba
ros2 topic pub /fix sensor_msgs/NavSatFix "{
  latitude: <LAT_DENTRO_DEL_MAPA>,
  longitude: <LON_DENTRO_DEL_MAPA>,
  altitude: 100.0,
  status: {status: 0, service: 1}
}" --once

# Seleccionar objetivo en web interface
# El sistema debe calcular y mostrar una ruta
```

## 🐛 Solución de Problemas Comunes

### Problema 1: "Failed to load Lanelet2 map"

**Causas posibles:**
- Archivo no existe en la ruta especificada
- Formato OSM inválido
- Tags Lanelet2 faltantes o incorrectos

**Solución:**
```bash
# Verificar archivo existe
ls -lh /workspace/config/mi_mapa_personalizado.osm

# Validar XML
xmllint --noout /workspace/config/mi_mapa_personalizado.osm

# Verificar tags Lanelet2
grep 'type="lanelet"' /workspace/config/mi_mapa_personalizado.osm
```

### Problema 2: "No lanelets loaded" (num_lanelets: 0)

**Causa:** Mapa no tiene relaciones tipo lanelet

**Solución:**
1. Abrir mapa en JOSM con plugin Lanelet2
2. Verificar que existen relaciones con `type=lanelet`
3. Si no existen, crear lanelets según Paso 3 del Método 1

### Problema 3: "No path found using Lanelet2 routing"

**Causas posibles:**
- Posición GPS fuera del área del mapa
- Lanelets no conectados (nodos no compartidos)
- Sentido de circulación incorrecto (one_way)

**Solución:**
```bash
# Verificar coordenadas GPS están dentro del mapa
# Lat/Lon deben estar dentro de bounds del mapa

# Ver bounds en logs:
docker-compose logs ros2 | grep bounds

# Verificar en JOSM:
# 1. Seleccionar herramienta de validación
# 2. Buscar "unconnected nodes"
# 3. Conectar lanelets compartiendo nodos
```

### Problema 4: Mapa muy grande/lento

**Causa:** Demasiados lanelets o resolución muy alta

**Solución:**
- Reducir área del mapa (<0.5 km²)
- Simplificar geometría (menos nodos por línea)
- Usar `map_resolution` más bajo en configuración

## 📝 Checklist de Datos Mínimos Requeridos

Para que un mapa funcione correctamente, debe tener:

- [ ] **Archivo OSM válido** (XML bien formado)
- [ ] **Al menos un lanelet** (relación con type=lanelet)
- [ ] **Líneas de borde** (left y right) para cada lanelet
- [ ] **Tags obligatorios** en cada lanelet:
  - [ ] type=lanelet
  - [ ] subtype (road, highway, etc.)
  - [ ] location (urban, rural, etc.)
- [ ] **Nodos compartidos** entre lanelets conectados
- [ ] **Coordenadas GPS válidas** (dentro de rangos razonables)
- [ ] **Archivo ubicado** en carpeta `config/`

## 📊 Ejemplo de Estructura Mínima

```xml
<?xml version='1.0' encoding='UTF-8'?>
<osm version="0.6" generator="JOSM">
  <!-- Nodos -->
  <node id="1" lat="48.984" lon="8.390" version="1"/>
  <node id="2" lat="48.984" lon="8.391" version="1"/>
  <node id="3" lat="48.985" lon="8.390" version="1"/>
  <node id="4" lat="48.985" lon="8.391" version="1"/>

  <!-- Líneas de borde -->
  <way id="10" version="1">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="solid"/>
  </way>

  <way id="11" version="1">
    <nd ref="3"/>
    <nd ref="4"/>
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="solid"/>
  </way>

  <!-- Lanelet -->
  <relation id="100" version="1">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="location" v="urban"/>
    <tag k="one_way" v="yes"/>
    <tag k="speed_limit" v="50"/>
  </relation>
</osm>
```

## 🔗 Recursos Adicionales

### Documentación Oficial
- [Lanelet2 GitHub](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [Lanelet2 Paper](https://arxiv.org/abs/1809.10728)
- [JOSM Documentation](https://josm.openstreetmap.de/wiki)

### Tutoriales
- [Lanelet2 Mapping Guide](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_maps/README.md)
- [Creating Lanelet2 Maps Tutorial](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_examples)

### Mapas de Ejemplo
- [Lanelet2 Example Maps](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_maps/res)

### Herramientas Útiles
- **JOSM:** https://josm.openstreetmap.de/
- **Lanelet2 Python:** `pip install lanelet2`
- **Validador OSM:** https://validator.openstreetmap.org/

## 💡 Consejos y Buenas Prácticas

1. **Empezar pequeño:** Crea un mapa de área reducida (100x100m) para probar
2. **Usar capas:** En JOSM, organiza elementos en capas diferentes
3. **Validar frecuentemente:** Ejecuta validación después de cada cambio
4. **Guardar versiones:** Mantén backups del mapa durante desarrollo
5. **Documentar:** Añade notas sobre características especiales del área
6. **Probar iterativamente:** Carga → Prueba → Ajusta → Repite
7. **Consultar ejemplos:** Estudia mapas de ejemplo oficiales de Lanelet2

## 🎯 Resumen Rápido

Para cargar un mapa personalizado:

1. **Crear/obtener mapa Lanelet2** (OSM con tags específicos)
2. **Colocar en** `config/mi_mapa.osm`
3. **Configurar** en `docker-compose.yml` o launch file
4. **Publicar GPS** dentro del área del mapa
5. **Verificar** logs y metadatos
6. **Probar** enrutamiento en web interface

---

**¿Necesitas ayuda?** Consulta también:
- [LANELET2_INTEGRATION.md](LANELET2_INTEGRATION.md) - Guía técnica completa
- [README.md](README.md) - Documentación general del proyecto
