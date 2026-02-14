# Configuración de Mapas - Guía Rápida

## 🎯 Problema Resuelto

Este documento describe la solución implementada para el problema de carga de mapas desde la carpeta `config/`.

### Problema Original

Los mapas colocados en la carpeta `config/` no se cargaban automáticamente porque:
- El parámetro `osm_file` no se pasaba al launch file
- El sistema siempre usaba el valor por defecto (cadena vacía)
- Esto causaba que se creara un mapa por defecto en lugar de cargar el archivo OSM

### Solución Implementada

Se implementaron las siguientes mejoras:

1. **Variable de entorno `OSM_FILE`** en docker-compose.yml
2. **Valor por defecto** apuntando a `sample_map.osm`
3. **Documentación clara** de cómo cambiar el mapa
4. **Múltiples métodos** de configuración para mayor flexibilidad

## 🚀 Cómo Usar Mapas Personalizados

### Método 1: Archivo .env (Recomendado) ✅

Este es el método más limpio y recomendado:

```bash
# 1. Copiar el archivo de configuración de ejemplo
cp config/example.env .env

# 2. Editar .env y cambiar la línea OSM_FILE
nano .env
# Cambiar a: OSM_FILE=/workspace/config/tu_mapa.osm

# 3. Colocar tu mapa en la carpeta config
cp tu_mapa.osm config/

# 4. Iniciar el sistema
docker compose up
```

**Ventajas:**
- ✅ No requiere editar archivos del proyecto
- ✅ El archivo .env es ignorado por git
- ✅ Fácil de cambiar entre mapas
- ✅ No requiere rebuild

### Método 2: Variable de Entorno Directa

Para pruebas rápidas o cambios temporales:

```bash
# Especificar el mapa al iniciar
OSM_FILE=/workspace/config/dekra.osm docker compose up
```

**Ventajas:**
- ✅ No requiere editar archivos
- ✅ Perfecto para pruebas rápidas
- ✅ No requiere rebuild

### Método 3: Editar docker-compose.yml

Para configuración permanente en el proyecto:

```yaml
services:
  ros2:
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - OSM_FILE=/workspace/config/tu_mapa.osm  # <-- Cambiar aquí
```

**Ventajas:**
- ✅ Configuración permanente
- ✅ Compartida por todo el equipo

**Desventajas:**
- ❌ Requiere commit si se quiere compartir
- ❌ Puede causar conflictos si cada usuario usa mapas diferentes

## 📍 Mapas Disponibles por Defecto

El proyecto incluye dos mapas de ejemplo:

1. **sample_map.osm** (por defecto)
   - Mapa pequeño de ejemplo
   - Ideal para pruebas iniciales
   - 1.9 KB

2. **dekra.osm**
   - Mapa más completo del área DEKRA
   - 1.2 MB
   - Contiene más carriles y elementos

Para usar dekra.osm:
```bash
OSM_FILE=/workspace/config/dekra.osm docker compose up
```

## 🔍 Verificar Configuración

### 1. Ver la configuración actual

```bash
docker compose config | grep OSM_FILE
```

Debería mostrar:
```
OSM_FILE: /workspace/config/sample_map.osm
```

### 2. Verificar que el mapa se carga

```bash
# Iniciar el sistema
docker compose up

# En otra terminal, ver los logs
docker compose logs ros2 | grep -i "loading.*map"
```

Deberías ver algo como:
```
Loading Lanelet2 map from: /workspace/config/sample_map.osm
Loaded X lanelets and Y areas
```

### 3. Verificar metadatos del mapa

```bash
# Ver el topic de metadatos
docker compose exec ros2 bash -c "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && ros2 topic echo /map_metadata --once"
```

## 📝 Añadir Tu Propio Mapa

### Paso 1: Preparar el mapa

Tu mapa debe ser un archivo OSM con formato Lanelet2. Ver [GUIA_MAPAS_PERSONALIZADOS.md](GUIA_MAPAS_PERSONALIZADOS.md) para crear uno.

### Paso 2: Colocar el mapa

```bash
# Copiar tu mapa a la carpeta config
cp mi_mapa_personalizado.osm config/
```

### Paso 3: Configurar

```bash
# Usar método .env (recomendado)
echo "OSM_FILE=/workspace/config/mi_mapa_personalizado.osm" >> .env

# O usar variable directa
OSM_FILE=/workspace/config/mi_mapa_personalizado.osm docker compose up
```

## 🐛 Solución de Problemas

### El mapa no se carga

**Síntoma:** Los logs muestran "OSM file not found" o se crea un mapa por defecto

**Soluciones:**

1. Verificar que el archivo existe:
```bash
ls -lh config/mi_mapa.osm
```

2. Verificar que la ruta es correcta (debe ser `/workspace/config/...`):
```bash
docker compose config | grep OSM_FILE
```

3. Verificar los logs:
```bash
docker compose logs ros2 | grep -i "osm\|map"
```

### Error "Failed to load Lanelet2 map"

**Causa:** El archivo OSM no tiene el formato correcto de Lanelet2

**Solución:**
- Verificar que el mapa tiene etiquetas Lanelet2 (type=lanelet)
- Ver [GUIA_MAPAS_PERSONALIZADOS.md](GUIA_MAPAS_PERSONALIZADOS.md) para validar el formato

### Los cambios no se aplican

**Causa:** El contenedor está usando configuración antigua

**Solución:**
```bash
# Detener y reconstruir
docker compose down
docker compose up --build
```

## 📚 Referencias

- [README.md](README.md) - Documentación general del proyecto
- [GUIA_MAPAS_PERSONALIZADOS.md](GUIA_MAPAS_PERSONALIZADOS.md) - Cómo crear mapas Lanelet2
- [LANELET2_INTEGRATION.md](LANELET2_INTEGRATION.md) - Integración técnica de Lanelet2

## ✅ Checklist de Verificación

Usa esta checklist para verificar que todo funciona:

- [ ] Los mapas están en la carpeta `config/`
- [ ] La variable `OSM_FILE` está configurada (en .env o docker-compose.yml)
- [ ] El sistema inicia sin errores: `docker compose up`
- [ ] Los logs muestran "Loading Lanelet2 map from..."
- [ ] Los logs muestran "Loaded X lanelets..."
- [ ] La interfaz web muestra el mapa: http://localhost:8000
- [ ] El topic `/map_metadata` contiene información del mapa

---

**Fecha de actualización:** 2026-02-14
**Versión:** 1.0
