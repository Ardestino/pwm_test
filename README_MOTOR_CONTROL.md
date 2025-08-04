# Robot Motor Control System

Sistema de control de motores paso a paso con ESP32 usando MCPWM.

## Configuración WiFi

Para habilitar la interfaz web, configura tus credenciales WiFi en el archivo `main/wifi_config.h`:

```c
#define WIFI_SSID "TU_RED_WIFI"
#define WIFI_PASS "TU_CONTRASEÑA_WIFI"
```

## Interfaces Disponibles

### 1. UART (Siempre disponible)

- Puerto: USB (115200 baud)
- Comandos de texto simples

### 2. Web Interface (Requiere WiFi)

- Acceso: `http://[IP_del_ESP32]/`
- API REST en `/api/command` y `/api/status`

## Comandos Disponibles

| Comando            | Descripción                | Ejemplo                   |
| ------------------ | -------------------------- | ------------------------- |
| `MOVE x y z [t]`   | Movimiento relativo        | `MOVE 100 -50 25 2.0`     |
| `MOVETO x y z [t]` | Movimiento absoluto        | `MOVETO 1500 300 100 3.0` |
| `POS`              | Obtener posición actual    | `POS`                     |
| `RESET`            | Reset posición a [0,0,0]   | `RESET`                   |
| `SETPOS x y z`     | Establecer posición        | `SETPOS 0 0 0`            |
| `LIMITS`           | Mostrar límites de motores | `LIMITS`                  |
| `STATUS`           | Estado del sistema         | `STATUS`                  |
| `STOP`             | Parada de emergencia       | `STOP`                    |

## Límites de Motores

- Motor 0 (X): 0 - 3200 pasos
- Motor 1 (Y): 0 - 2100 pasos
- Motor 2 (Z): 0 - 700 pasos

## Pines de Hardware

### Motores Paso a Paso

- Motor 0: STEP=27, DIR=26
- Motor 1: STEP=33, DIR=32
- Motor 2: STEP=19, DIR=21

## Compilación

```bash
cd pwm_test
idf.py build
idf.py flash monitor
```

## API REST

### POST /api/command

Envía comandos al sistema.

**Ejemplo:**

```bash
curl -X POST http://192.168.1.100/api/command \
     -H "Content-Type: text/plain" \
     -d "MOVE 100 50 25 2.0"
```

### GET /api/status

Obtiene el estado actual del sistema.

**Respuesta:**

```json
{
  "status": "ok",
  "last_response": "Position: [1500,300,100]"
}
```

## Características

- ✅ Control sincronizado de 3 motores
- ✅ Límites de seguridad por motor
- ✅ Interfaz UART y Web
- ✅ API REST completa
- ✅ Rastreo de posición absoluta
- ✅ Página web responsive
- ✅ Configuración WiFi externa
- ✅ Timeouts y manejo de errores
- ✅ Watchdog timer protection
