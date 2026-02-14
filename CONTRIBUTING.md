# Contributing to Planner Map

¡Gracias por tu interés en contribuir! Este documento proporciona guías para contribuir al proyecto.

## 🚀 Guía Rápida

1. Fork el repositorio
2. Crea una rama para tu feature: `git checkout -b feature/nueva-funcionalidad`
3. Haz tus cambios
4. Ejecuta las pruebas
5. Commit tus cambios: `git commit -m 'Agregar nueva funcionalidad'`
6. Push a tu fork: `git push origin feature/nueva-funcionalidad`
7. Abre un Pull Request

## 🏗️ Estructura del Proyecto

### ROS2 Package
```
ros2_ws/src/planner_map/
├── planner_map/          # Código Python
│   ├── planner_node.py   # Nodo de planificación
│   ├── map_server.py     # Servidor de mapas
│   └── ros2_web_bridge.py # Bridge ROS2-Web
├── launch/               # Launch files
├── config/               # Archivos de configuración
├── package.xml           # Metadatos del paquete
└── setup.py             # Setup Python
```

### Web Interface
```
web_interface/
├── main.py              # Aplicación FastAPI
├── static/              # Assets estáticos
│   ├── app.js          # JavaScript
│   └── style.css       # Estilos
├── templates/           # Templates HTML
└── requirements.txt     # Dependencias Python
```

## 🛠️ Configuración del Entorno de Desarrollo

### Opción 1: Con Docker (Recomendado)

```bash
# Usar archivo de desarrollo
docker compose -f docker compose.dev.yml up --build

# O usar el Makefile
make dev
```

### Opción 2: Local (Sin Docker)

#### ROS2
```bash
# Instalar ROS2 Humble
# https://docs.ros.org/en/humble/Installation.html

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

#### Web Interface
```bash
cd web_interface
pip install -r requirements.txt
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

## 📝 Estándares de Código

### Python
- Seguir PEP 8
- Usar type hints cuando sea posible
- Documentar funciones con docstrings
- Máximo 100 caracteres por línea

```python
def example_function(param: str) -> bool:
    """
    Descripción breve de la función.
    
    Args:
        param: Descripción del parámetro
        
    Returns:
        Descripción del valor de retorno
    """
    pass
```

### ROS2
- Seguir las convenciones de ROS2
- Usar logging apropiado (`self.get_logger()`)
- Limpiar recursos en `__del__` o `destroy_node()`

### JavaScript
- Usar `const` y `let`, no `var`
- Nombres descriptivos para variables y funciones
- Comentar código complejo

## 🧪 Testing

### Python
```bash
cd web_interface
pytest tests/
```

### ROS2
```bash
cd ros2_ws
colcon test
```

## 📦 Agregar Dependencias

### Python (Web)
```bash
# Agregar a web_interface/requirements.txt
cd web_interface
echo "nueva-libreria==1.0.0" >> requirements.txt
pip install -r requirements.txt
```

### ROS2
```bash
# Agregar a package.xml
<depend>nuevo_paquete</depend>

# O para Python
echo "nueva-libreria==1.0.0" >> ros2_ws/requirements.txt
```

## 🐛 Reportar Bugs

Al reportar un bug, incluye:

1. Descripción clara del problema
2. Pasos para reproducir
3. Comportamiento esperado vs actual
4. Versiones de software (ROS2, Docker, etc.)
5. Logs relevantes

## ✨ Sugerir Features

Para sugerir nuevas funcionalidades:

1. Revisa los issues existentes
2. Crea un nuevo issue con etiqueta "enhancement"
3. Describe claramente el caso de uso
4. Proporciona ejemplos si es posible

## 📚 Documentación

- Actualiza el README si cambias funcionalidad
- Documenta nuevos endpoints de API
- Agrega comentarios en código complejo
- Actualiza diagramas si cambias arquitectura

## 🔄 Proceso de Pull Request

1. **Actualiza tu fork**
   ```bash
   git fetch upstream
   git merge upstream/main
   ```

2. **Asegúrate de que pasan las pruebas**
   ```bash
   make test
   ```

3. **Commit con mensajes descriptivos**
   ```bash
   git commit -m "feat: agregar endpoint para nuevo tipo de mapa"
   ```

4. **Push y crea PR**
   ```bash
   git push origin feature/nueva-funcionalidad
   ```

5. **Describe tu PR**
   - Qué cambia
   - Por qué es necesario
   - Cómo probarlo

## 🎯 Áreas para Contribuir

- 🤖 Algoritmos de planificación más avanzados
- 🗺️ Soporte para diferentes tipos de mapas
- 🎨 Mejoras en la UI/UX
- 📊 Visualizaciones adicionales
- 🧪 Tests adicionales
- 📝 Documentación
- 🐛 Corrección de bugs

## ❓ ¿Necesitas Ayuda?

- Abre un issue con etiqueta "question"
- Revisa la documentación en README.md
- Contacta a los maintainers

## 📄 Licencia

Al contribuir, aceptas que tus contribuciones se licencien bajo Apache-2.0.
