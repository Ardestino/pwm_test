# Robot Motor Control - API de Python

Una API de Python completa y simplificada para controlar el robot de motores paso a paso ESP32 a través de HTTP REST.

## 📁 Estructura del Directorio

```
python_api/
├── __init__.py           # Inicialización del paquete Python
├── robot_api.py          # API principal del robot
├── examples.py           # Ejemplos de uso y patrones
├── quick_test.py         # Prueba rápida de conectividad
├── requirements.txt      # Dependencias de Python
└── README.md            # Esta documentación
```

## 🚀 Instalación Rápida

1. **Navegar al directorio:**

```bash
cd c:\Users\ramon\Desktop\pwm_test\python_api
```

2. **Instalar dependencias:**

```bash
pip install -r requirements.txt
```

3. **Verificar conexión:**

```bash
python quick_test.py
```

4. **Ejecutar ejemplos:**

```bash
python examples.py
```

## 🔧 Uso desde Proyecto Principal

### Opción 1: Usar como paquete local

```python
# Desde el directorio pwm_test
from python_api import create_robot_connection, Position

with create_robot_connection("192.168.100.72") as robot:
    robot.move_to(100, 200, 50, 2.0)
    print(f"Posición: {robot.get_position()}")
```

### Opción 2: Usar directamente desde python_api

```bash
cd python_api
python
```

```python
from robot_api import create_robot_connection

with create_robot_connection("192.168.100.72") as robot:
    robot.home()
```

## 📖 Archivos Principales

### `robot_api.py`

Contiene la API principal con todas las clases y funciones:

- **RobotAPI**: Clase principal para controlar el robot
- **Position**: Manejo de coordenadas 3D
- **MotorLimits**: Información de límites de motores
- **Funciones de utilidad**: Conexión y auto-descubrimiento

### `examples.py`

Ejemplos prácticos organizados por categorías:

1. **Movimientos Básicos**: Relativos y absolutos
2. **Movimientos de Jog**: Control de ejes individuales
3. **Patrones Geométricos**: Formas y trayectorias
4. **Movimientos de Precisión**: Micro-posicionamiento
5. **Control Interactivo**: Teclado WASD
6. **Prueba de Estrés**: Movimientos masivos

### `quick_test.py`

Herramienta de diagnóstico que:

- Busca automáticamente el robot en la red
- Verifica conectividad HTTP
- Prueba comandos básicos
- Reporta el estado del sistema

## 🎮 Ejemplos de Uso Rápido

### Movimientos Básicos

```python
from robot_api import create_robot_connection

with create_robot_connection("192.168.100.72") as robot:
    # Movimiento relativo
    robot.move_relative(100, -50, 25, 1.5)

    # Movimiento absoluto
    robot.move_to(500, 300, 150, 2.0)

    # Jog individual
    robot.jog('x', 100, 1.0)

    # Información
    pos = robot.get_position()
    limits = robot.get_limits()
```

### Secuencias Avanzadas

```python
# Patrón cuadrado
sequence = [
    (0, 0, 0, 1.0),
    (200, 0, 0, 1.5),
    (200, 200, 0, 1.5),
    (0, 200, 0, 1.5),
    (0, 0, 0, 1.5)
]
robot.move_sequence(sequence, wait_between=0.5)

# Patrón circular
import math
center_x, center_y, radius = 400, 300, 100

for i in range(8):
    angle = i * (2 * math.pi / 8)
    x = center_x + int(radius * math.cos(angle))
    y = center_y + int(radius * math.sin(angle))
    robot.move_to(x, y, 50, 1.0)
```

### Control Interactivo

```bash
python examples.py
# Seleccionar opción 5: Control Interactivo

Comandos disponibles:
  w/s: Mover Y +/-
  a/d: Mover X -/+
  q/e: Mover Z +/-
  h: Ir al origen
  p: Mostrar posición
  r: Reset posición
  x: Salir
```

## 🛡️ Características de Seguridad

- **Validación automática de límites** antes de cada movimiento
- **Manejo robusto de errores** de conexión y comandos
- **Timeouts configurables** para evitar bloqueos
- **Verificación de posiciones** antes de mover
- **Parada de emergencia** disponible

## 🌐 Auto-descubrimiento de Red

La API puede encontrar automáticamente tu robot:

```python
from robot_api import discover_robot, create_robot_connection

# Buscar en diferentes redes
ip = discover_robot("192.168.100") or discover_robot("192.168.1")

if ip:
    print(f"Robot encontrado en: {ip}")
    with create_robot_connection(ip) as robot:
        robot.move_to(100, 100, 50, 2.0)
else:
    print("No se encontró robot en la red")
```

## 📊 Monitoreo y Debugging

### Logging Detallado

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Ahora verás información detallada de todas las operaciones
```

### Información del Sistema

```python
with create_robot_connection("192.168.100.72") as robot:
    # Estado del robot
    status = robot.get_status_info()
    print(f"Estado: {status}")

    # Posición actual
    pos = robot.get_position()
    print(f"Posición: {pos}")

    # Límites de motores
    limits = robot.get_limits()
    print(f"Límites: {limits}")

    # Verificar seguridad
    safe = robot.is_position_safe(1000, 500, 200)
    print(f"Posición segura: {safe}")
```

## 🚨 Solución de Problemas

### Error: "No se puede conectar al robot"

```bash
# 1. Verificar conectividad básica
python quick_test.py

# 2. Buscar robot automáticamente
python -c "from robot_api import discover_robot; print(discover_robot('192.168.100'))"

# 3. Verificar manualmente
ping 192.168.100.72
```

### Error: "Error de comando"

- Verifica que las posiciones estén dentro de los límites
- Confirma que la duración sea mayor a 0
- Usa `robot.get_limits()` para ver rangos válidos

### Configuración de IP

Si tu robot está en una IP diferente, modifica la constante en `examples.py`:

```python
# En examples.py línea ~280
ROBOT_IP = "192.168.1.100"  # Cambiar según tu red
```

## 🔗 Integración con Otros Proyectos

### Como submódulo

```bash
# Desde tu proyecto
git submodule add <repo-url>/pwm_test/python_api robot_control
```

### Como paquete local

```python
import sys
sys.path.append('path/to/pwm_test')
from python_api import create_robot_connection
```

## 📈 Próximas Características

- [ ] **GUI con tkinter**: Interfaz gráfica de usuario
- [ ] **Guardado de trayectorias**: Grabar y reproducir secuencias
- [ ] **Interpolación avanzada**: Movimientos suaves entre puntos
- [ ] **Logging a archivo**: Registro histórico de movimientos
- [ ] **Configuración JSON**: Parámetros externos configurables
- [ ] **Simulación offline**: Pruebas sin hardware

## 📞 Soporte

1. **Prueba rápida**: `python quick_test.py`
2. **Ejemplos**: `python examples.py`
3. **Monitor ESP32**: Revisa logs del ESP32 para errores de hardware
4. **Red**: Verifica que ESP32 y PC estén en la misma red WiFi

## 📄 Licencia

Este código es parte del proyecto Robot Motor Control ESP32.
Libre uso para proyectos educativos y de desarrollo.
