# Robot Motor Control - API de Python

Una API de Python simplificada para controlar el robot de motores paso a paso ESP32 a través de HTTP REST.

## 📋 Características

- **Conexión HTTP**: Comunicación con el ESP32 a través de WiFi
- **API Simplificada**: Funciones fáciles de usar para control del robot
- **Movimientos Avanzados**: Soporte para patrones, secuencias y jog
- **Gestión de Errores**: Manejo robusto de errores de conexión y comandos
- **Auto-descubrimiento**: Búsqueda automática del robot en la red
- **Validación de Seguridad**: Verificación de límites antes de mover

## 🚀 Instalación Rápida

1. **Instalar dependencias:**

```bash
pip install -r requirements.txt
```

2. **Verificar conexión:**

```bash
python quick_test.py
```

3. **Ejecutar ejemplos:**

```bash
python examples.py
```

## 🔧 Uso Básico

### Conexión Simple

```python
from robot_api import create_robot_connection

# Conexión directa con IP conocida
with create_robot_connection("192.168.100.72") as robot:
    robot.move_to(100, 200, 50, 2.0)
    print(f"Posición: {robot.get_position()}")
```

### Auto-descubrimiento

```python
from robot_api import discover_robot, create_robot_connection

# Buscar robot automáticamente
ip = discover_robot("192.168.100")
if ip:
    with create_robot_connection(ip) as robot:
        robot.home()  # Ir al origen
```

## 🎮 Funciones Principales

### Movimientos Básicos

```python
# Movimiento relativo (desde posición actual)
robot.move_relative(x=100, y=-50, z=25, duration=1.5)

# Movimiento absoluto (a posición específica)
robot.move_to(x=500, y=300, z=150, duration=2.0)

# Movimiento de un solo eje (jog)
robot.jog('x', steps=100, duration=1.0)  # Mover X +100
robot.jog('y', steps=-50, duration=0.8)  # Mover Y -50
```

### Información del Robot

```python
# Obtener posición actual
pos = robot.get_position()
print(f"X: {pos.x}, Y: {pos.y}, Z: {pos.z}")

# Obtener límites de movimiento
limits = robot.get_limits()
print(f"Límites: {limits}")

# Verificar si una posición es segura
safe = robot.is_position_safe(1000, 500, 200)
```

### Control del Robot

```python
# Ir al origen
robot.home(duration=3.0)

# Resetear posición a [0,0,0]
robot.reset_position()

# Establecer posición actual manualmente
robot.set_position(100, 200, 50)

# Parada de emergencia
robot.emergency_stop()
```

### Movimientos Avanzados

```python
# Secuencia de movimientos
sequence = [
    (100, 100, 50, 1.0),   # (x, y, z, duration)
    (200, 150, 75, 1.5),
    (50, 200, 100, 2.0)
]
robot.move_sequence(sequence, wait_between=0.5)

# Patrón circular
import math
center_x, center_y = 400, 300
radius = 100

for i in range(8):
    angle = i * (2 * math.pi / 8)
    x = center_x + int(radius * math.cos(angle))
    y = center_y + int(radius * math.sin(angle))
    robot.move_to(x, y, 50, 1.0)
```

## 📖 Ejemplos Incluidos

### `examples.py`

Ejecuta `python examples.py` para acceder a:

1. **Movimientos Básicos**: Pruebas de movimientos relativos y absolutos
2. **Movimientos de Jog**: Control de ejes individuales
3. **Patrones Geométricos**: Cuadrados, hélices y formas
4. **Movimientos de Precisión**: Micro-posicionamiento y verificación de límites
5. **Control Interactivo**: Control manual por teclado
6. **Prueba de Estrés**: Múltiples movimientos aleatorios

### Control Interactivo

```
Comandos disponibles:
  w/s: Mover Y +/-
  a/d: Mover X -/+
  q/e: Mover Z +/-
  h: Ir al origen
  p: Mostrar posición
  r: Reset posición
  x: Salir
```

## 🛡️ Manejo de Errores

La API maneja automáticamente:

- **Errores de conexión**: WiFi desconectado, IP incorrecta
- **Errores de comandos**: Comandos inválidos o posiciones fuera de límites
- **Timeouts**: Conexiones que no responden
- **Validación de límites**: Previene movimientos peligrosos

```python
from robot_api import RobotConnectionError, RobotCommandError

try:
    with create_robot_connection("192.168.100.72") as robot:
        robot.move_to(1000, 500, 200, 2.0)
except RobotConnectionError as e:
    print(f"Error de conexión: {e}")
except RobotCommandError as e:
    print(f"Error de comando: {e}")
```

## 🔧 Configuración Avanzada

### Timeout Personalizado

```python
robot = create_robot_connection("192.168.100.72", timeout=15.0)
```

### Logging Detallado

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 📝 Referencia de API

### Clase `RobotAPI`

#### Movimientos

- `move_relative(x, y, z, duration)` - Movimiento relativo
- `move_to(x, y, z, duration)` - Movimiento absoluto
- `move_to_position(position, duration)` - Movimiento con objeto Position
- `jog(axis, steps, duration)` - Movimiento de un eje
- `home(duration)` - Ir al origen
- `move_sequence(positions, wait_between)` - Secuencia de movimientos

#### Información

- `get_position()` - Posición actual (retorna `Position`)
- `get_limits()` - Límites de motores (retorna `MotorLimits`)
- `get_status_info()` - Estado del sistema
- `is_position_safe(x, y, z)` - Verificar límites
- `get_distance_to(x, y, z)` - Distancia euclidiana

#### Control

- `reset_position()` - Reset a [0,0,0]
- `set_position(x, y, z)` - Establecer posición actual
- `emergency_stop()` - Parada de emergencia
- `wait_for_movement(timeout)` - Esperar fin de movimiento

### Clase `Position`

```python
pos = Position(x=100, y=200, z=50)
print(f"X: {pos.x}, Y: {pos.y}, Z: {pos.z}")
coords = pos.to_list()  # [100, 200, 50]
```

### Clase `MotorLimits`

```python
limits = robot.get_limits()
print(f"Motor 0: [{limits.min_limits[0]}, {limits.max_limits[0]}]")
safe = limits.is_position_valid(Position(500, 300, 100))
```

## 🌐 Configuración de Red

Asegúrate de que:

1. El ESP32 esté conectado al WiFi
2. Tu computadora esté en la misma red
3. El firewall permita conexiones HTTP al puerto 80

Para verificar la IP del ESP32, revisa el monitor serie o tu router.

## 🐛 Solución de Problemas

### "No se puede conectar al robot"

- Verifica que el ESP32 esté encendido
- Confirma que esté conectado al WiFi (LED indicador)
- Usa `discover_robot()` para encontrar la IP automáticamente
- Verifica que no haya firewall bloqueando

### "Error de comando"

- Verifica que la posición esté dentro de los límites
- Confirma que la duración sea mayor a 0
- Usa `get_limits()` para verificar rangos válidos

### "Timeout"

- Reduce la velocidad de movimientos
- Aumenta el timeout en la conexión
- Verifica la estabilidad del WiFi

## 📞 Soporte

Para problemas o mejoras:

1. Verifica el log del ESP32 en el monitor serie
2. Usa `quick_test.py` para diagnóstico básico
3. Revisa los ejemplos en `examples.py`

## 🚀 Próximas Características

- [ ] Interpolación de trayectorias
- [ ] Guardado/carga de secuencias
- [ ] Interface gráfica con tkinter
- [ ] Registro de movimientos en archivo
- [ ] Control de velocidad avanzado
