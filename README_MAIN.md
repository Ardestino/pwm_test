# Robot Motor Control ESP32

Un sistema completo de control de motores paso a paso para ESP32 con interfaz web y API de Python.

## 📁 Estructura del Proyecto

```
pwm_test/
├── main/                     # Código ESP32 (C++)
│   ├── test.cpp             # Sistema principal de control de motores
│   ├── main.cpp             # Ejemplo básico MCPWM
│   ├── wifi_config.h        # Configuración WiFi
│   └── CMakeLists.txt       # Configuración de build
├── python_api/              # 🆕 API de Python
│   ├── robot_api.py         # API principal del robot
│   ├── examples.py          # Ejemplos y patrones de uso
│   ├── quick_test.py        # Prueba rápida de conectividad
│   ├── requirements.txt     # Dependencias Python
│   └── README.md           # Documentación de la API
├── build/                   # Archivos de compilación ESP-IDF
├── CMakeLists.txt          # Configuración principal
└── README.md               # Esta documentación
```

## 🚀 Características Principales

### 🔧 ESP32 (Hardware)

- **Control sincronizado** de 3 motores paso a paso
- **PWM de alta precisión** (1MHz) usando MCPWM
- **Control de dirección** con pines GPIO dedicados
- **Límites de seguridad** configurables por motor
- **Seguimiento de posición** en tiempo real
- **Interfaz web** con panel de control HTML5
- **API REST** para comandos remotos
- **Interfaz UART** para control por serie

### 🐍 Python API (Software)

- **API simplificada** para control desde PC
- **Auto-descubrimiento** de robot en red
- **Patrones de movimiento** predefinidos
- **Control interactivo** por teclado
- **Validación de seguridad** automática
- **Manejo robusto de errores**
- **Ejemplos completos** incluidos

## 🔌 Conexiones Hardware

### Motores Paso a Paso

| Motor       | Pin STEP | Pin DIR | Límites (pasos) |
| ----------- | -------- | ------- | --------------- |
| Motor 0 (X) | GPIO 27  | GPIO 26 | 0 - 3200        |
| Motor 1 (Y) | GPIO 33  | GPIO 32 | 0 - 2100        |
| Motor 2 (Z) | GPIO 19  | GPIO 21 | 0 - 700         |

### Configuración Típica

```
Driver Paso a Paso → ESP32
VCC     → 3.3V/5V
GND     → GND
STEP    → GPIO (27, 33, 19)
DIR     → GPIO (26, 32, 21)
ENABLE  → GND (siempre habilitado)
```

## 🛠️ Instalación ESP32

### 1. Configurar ESP-IDF

```bash
# Instalar ESP-IDF v5.4.1+
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
source export.sh
```

### 2. Configurar WiFi

```bash
# Crear archivo de configuración WiFi
cp main/wifi_config.h.example main/wifi_config.h
# Editar wifi_config.h con tus credenciales
```

### 3. Compilar y Flashear

```bash
idf.py build
idf.py -p COM3 flash monitor  # Windows
idf.py -p /dev/ttyUSB0 flash monitor  # Linux
```

## 🐍 Instalación API Python

### 1. Navegar al directorio

```bash
cd python_api
```

### 2. Instalar dependencias

```bash
pip install -r requirements.txt
```

### 3. Prueba rápida

```bash
python quick_test.py
```

## 🎮 Uso Básico

### Interfaz Web

1. Conectar ESP32 al WiFi
2. Obtener IP del monitor serie
3. Abrir navegador: `http://[ESP32_IP]/`
4. Usar panel de control web

### API Python

```python
from python_api import create_robot_connection

# Conectar al robot
with create_robot_connection("192.168.100.72") as robot:
    # Movimientos básicos
    robot.move_relative(100, 50, 25, 1.5)  # Relativo
    robot.move_to(500, 300, 150, 2.0)      # Absoluto
    robot.jog('x', 100, 1.0)               # Un eje
    robot.home(3.0)                        # Al origen

    # Información
    pos = robot.get_position()
    print(f"Posición: {pos}")
```

### Puerto Serie (UART)

```
Comandos disponibles:
MOVE x y z [duration]     - Movimiento relativo
MOVETO x y z [duration]   - Movimiento absoluto
POS                       - Posición actual
RESET                     - Reset a [0,0,0]
LIMITS                    - Mostrar límites
STATUS                    - Estado del sistema
STOP                      - Parada de emergencia
```

## 📖 Ejemplos y Tutoriales

### Python API

```bash
cd python_api
python examples.py
```

Ejemplos disponibles:

1. **Movimientos Básicos** - Relativos y absolutos
2. **Control de Jog** - Ejes individuales
3. **Patrones Geométricos** - Formas y trayectorias
4. **Movimientos de Precisión** - Micro-posicionamiento
5. **Control Interactivo** - Teclado WASD
6. **Prueba de Estrés** - Movimientos masivos

### Control Interactivo

```
Comandos de teclado:
w/s: Mover Y +/-
a/d: Mover X -/+
q/e: Mover Z +/-
h: Ir al origen
p: Mostrar posición
r: Reset posición
x: Salir
```

## 🔧 Configuración Avanzada

### Límites de Motores

Modifica en `test.cpp`:

```cpp
constexpr std::array<int32_t, MOTOR_COUNT> MIN_LIMITS = {0, 0, 0};
constexpr std::array<int32_t, MOTOR_COUNT> MAX_LIMITS = {3200, 2100, 700};
```

### Pines GPIO

Modifica en `test.cpp`:

```cpp
constexpr std::array<int, MOTOR_COUNT> STEP_PINS = {27, 33, 19};
constexpr std::array<int, MOTOR_COUNT> DIR_PINS = {26, 32, 21};
```

### Configuración WiFi

En `main/wifi_config.h`:

```c
#define WIFI_SSID "tu_red_wifi"
#define WIFI_PASS "tu_password"
#define WIFI_MAXIMUM_RETRY 5
```

## 🛡️ Características de Seguridad

- **Límites por motor**: Previene movimientos fuera de rango
- **Validación de comandos**: Verifica parámetros antes de ejecutar
- **Timeouts**: Evita bloqueos en comunicación
- **Parada de emergencia**: Detiene todos los motores inmediatamente
- **Seguimiento de posición**: Monitoreo continuo de ubicación

## 🌐 API REST

### Endpoints Disponibles

```
GET  /                    - Panel de control web
POST /api/command         - Enviar comando al robot
GET  /api/status          - Obtener estado actual
```

### Ejemplo de uso con curl

```bash
# Mover robot
curl -X POST http://192.168.100.72/api/command -d "MOVE 100 50 25 1.5"

# Obtener posición
curl -X POST http://192.168.100.72/api/command -d "POS"

# Obtener estado
curl http://192.168.100.72/api/status
```

## 🚨 Solución de Problemas

### ESP32 no se conecta al WiFi

1. Verificar credenciales en `wifi_config.h`
2. Revisar monitor serie para errores
3. Confirmar red WiFi 2.4GHz (no 5GHz)

### No se puede conectar desde Python

1. Ejecutar `python quick_test.py`
2. Verificar IP del ESP32 en monitor serie
3. Confirmar misma red WiFi
4. Probar ping manual: `ping [ESP32_IP]`

### Motores no se mueven

1. Verificar conexiones hardware
2. Confirmar alimentación de drivers
3. Revisar pines STEP/DIR en código
4. Verificar que comando esté dentro de límites

### Respuesta lenta de la web

1. Reducir número de conexiones simultáneas
2. Usar comandos más pequeños
3. Verificar estabilidad WiFi

## 📊 Monitoreo y Debug

### Monitor Serie ESP32

```bash
idf.py monitor
```

### Logging Python

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Información del Sistema

```python
with create_robot_connection("192.168.100.72") as robot:
    print(f"Posición: {robot.get_position()}")
    print(f"Límites: {robot.get_limits()}")
    print(f"Estado: {robot.get_status_info()}")
```

## 🔗 Recursos Adicionales

- **[ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/)**
- **[MCPWM Peripheral Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html)**
- **Python API**: Ver `python_api/README.md` para documentación detallada

## 📄 Arquitectura del Sistema

```
┌─────────────────┐    WiFi/HTTP     ┌──────────────────┐
│   Python API    │◄────────────────►│      ESP32       │
│                 │                  │                  │
│  - robot_api.py │                  │ - Web Interface  │
│  - examples.py  │                  │ - REST API       │
│  - quick_test   │                  │ - UART Interface │
└─────────────────┘                  │ - Motor Control  │
                                     └──────────────────┘
                                             │
                                             │ GPIO
                                             ▼
                                     ┌──────────────────┐
                                     │  Stepper Motors  │
                                     │                  │
                                     │  Motor 0 (X)     │
                                     │  Motor 1 (Y)     │
                                     │  Motor 2 (Z)     │
                                     └──────────────────┘
```

## 🚀 Próximas Características

- [ ] **GUI Python**: Interfaz gráfica con tkinter
- [ ] **Trayectorias suaves**: Interpolación entre puntos
- [ ] **Guardado de secuencias**: Grabar y reproducir movimientos
- [ ] **Control PID**: Retroalimentación de posición
- [ ] **Múltiples robots**: Control de varios ESP32
- [ ] **Simulación 3D**: Visualización virtual

---

**Proyecto desarrollado para control de motores paso a paso con ESP32 y API Python.**
