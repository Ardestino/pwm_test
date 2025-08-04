"""
Robot Motor Control - API de Python
===================================

API simplificada para controlar el robot de motores paso a paso ESP32.

Clases principales:
- RobotAPI: Interfaz principal para el control del robot
- Position: Representa una posición 3D
- MotorLimits: Información sobre límites de los motores

Funciones de utilidad:
- create_robot_connection(): Crea una conexión al robot
- discover_robot(): Busca automáticamente el robot en la red

Excepciones:
- RobotConnectionError: Errores de conexión
- RobotCommandError: Errores en comandos

Ejemplo de uso básico:
    from python_api import create_robot_connection
    
    with create_robot_connection("192.168.100.72") as robot:
        robot.move_to(100, 200, 50, 2.0)
        print(f"Posición: {robot.get_position()}")
"""

from .robot_api import (
    RobotAPI,
    Position,
    MotorLimits,
    RobotConnectionError,
    RobotCommandError,
    create_robot_connection,
    discover_robot
)

__version__ = "1.0.0"
__author__ = "Robot Control System"
__email__ = "robot@example.com"

__all__ = [
    'RobotAPI',
    'Position', 
    'MotorLimits',
    'RobotConnectionError',
    'RobotCommandError',
    'create_robot_connection',
    'discover_robot'
]
