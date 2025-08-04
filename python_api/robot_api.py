#!/usr/bin/env python3
"""
Robot Motor Control API - Cliente Python
========================================

API simplificada para controlar el robot de 3 motores paso a paso ESP32.
Comunicación a través de HTTP REST con el ESP32.

Autor: Sistema de Control de Motores
Fecha: Agosto 2025
"""

import requests
import json
import time
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
import logging

# Configurar logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


@dataclass
class Position:
    """Representa una posición 3D del robot"""
    x: int = 0
    y: int = 0
    z: int = 0
    
    def __iter__(self):
        return iter([self.x, self.y, self.z])
    
    def __getitem__(self, index):
        return [self.x, self.y, self.z][index]
    
    def __str__(self):
        return f"Position(x={self.x}, y={self.y}, z={self.z})"
    
    def to_list(self) -> List[int]:
        return [self.x, self.y, self.z]


@dataclass
class MotorLimits:
    """Representa los límites de cada motor"""
    min_limits: List[int]
    max_limits: List[int]
    
    def is_position_valid(self, pos: Position) -> bool:
        """Verifica si una posición está dentro de los límites"""
        for i, coord in enumerate(pos.to_list()):
            if coord < self.min_limits[i] or coord > self.max_limits[i]:
                return False
        return True
    
    def __str__(self):
        return f"Limits: Motor0[{self.min_limits[0]},{self.max_limits[0]}] Motor1[{self.min_limits[1]},{self.max_limits[1]}] Motor2[{self.min_limits[2]},{self.max_limits[2]}]"


class RobotConnectionError(Exception):
    """Excepción para errores de conexión con el robot"""
    pass


class RobotCommandError(Exception):
    """Excepción para errores en comandos del robot"""
    pass


class RobotAPI:
    """
    API principal para controlar el robot de motores paso a paso.
    
    Esta clase proporciona una interfaz Python simplificada para controlar
    el robot ESP32 a través de comandos HTTP REST.
    """
    
    def __init__(self, esp32_ip: str, port: int = 80, timeout: float = 10.0):
        """
        Inicializa la conexión con el robot.
        
        Args:
            esp32_ip: Dirección IP del ESP32
            port: Puerto HTTP (por defecto 80)
            timeout: Timeout para requests HTTP en segundos
        """
        self.base_url = f"http://{esp32_ip}:{port}"
        self.timeout = timeout
        self.session = requests.Session()
        self.session.headers.update({
            'Content-Type': 'text/plain',
            'User-Agent': 'RobotAPI-Python/1.0'
        })
        
        # Verificar conexión inicial
        self._verify_connection()
        logger.info(f"Conectado al robot en {self.base_url}")
    
    def _verify_connection(self) -> None:
        """Verifica que el robot esté accesible"""
        try:
            response = self.session.get(f"{self.base_url}/api/status", timeout=self.timeout)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            raise RobotConnectionError(f"No se puede conectar al robot: {e}")
    
    def _send_command(self, command: str) -> Dict[str, Any]:
        """
        Envía un comando al robot y retorna la respuesta.
        
        Args:
            command: Comando a enviar
            
        Returns:
            Diccionario con la respuesta del robot
            
        Raises:
            RobotConnectionError: Error de conexión
            RobotCommandError: Error en el comando
        """
        try:
            response = self.session.post(
                f"{self.base_url}/api/command",
                data=command,
                timeout=self.timeout
            )
            response.raise_for_status()
            
            result = response.json()
            if result.get('status') != 'ok':
                raise RobotCommandError(f"Error en comando '{command}': {result.get('message', 'Unknown error')}")
            
            logger.debug(f"Comando enviado: {command} -> {result.get('message', '')}")
            return result
            
        except requests.exceptions.Timeout:
            raise RobotConnectionError(f"Timeout al enviar comando: {command}")
        except requests.exceptions.RequestException as e:
            raise RobotConnectionError(f"Error de conexión: {e}")
        except json.JSONDecodeError as e:
            raise RobotCommandError(f"Respuesta inválida del robot: {e}")
    
    def _get_status(self) -> Dict[str, Any]:
        """Obtiene el estado actual del robot"""
        try:
            response = self.session.get(f"{self.base_url}/api/status", timeout=self.timeout)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            raise RobotConnectionError(f"Error obteniendo estado: {e}")
    
    # ==================== MOVIMIENTOS BÁSICOS ====================
    
    def move_relative(self, x: int = 0, y: int = 0, z: int = 0, duration: float = 1.0) -> bool:
        """
        Mueve el robot de forma relativa desde su posición actual.
        
        Args:
            x: Pasos a mover en el eje X (motor 0)
            y: Pasos a mover en el eje Y (motor 1) 
            z: Pasos a mover en el eje Z (motor 2)
            duration: Duración del movimiento en segundos
            
        Returns:
            True si el movimiento fue exitoso
            
        Example:
            robot.move_relative(100, -50, 25, 2.0)  # Mover X+100, Y-50, Z+25 en 2 segundos
        """
        command = f"MOVE {x} {y} {z} {duration}"
        self._send_command(command)
        logger.info(f"Movimiento relativo: [{x}, {y}, {z}] en {duration}s")
        return True
    
    def move_to(self, x: int, y: int, z: int, duration: float = 2.0) -> bool:
        """
        Mueve el robot a una posición absoluta.
        
        Args:
            x: Posición absoluta en X (motor 0)
            y: Posición absoluta en Y (motor 1)
            z: Posición absoluta en Z (motor 2)
            duration: Duración del movimiento en segundos
            
        Returns:
            True si el movimiento fue exitoso
            
        Example:
            robot.move_to(1000, 500, 200, 3.0)  # Ir a posición [1000, 500, 200] en 3 segundos
        """
        command = f"MOVETO {x} {y} {z} {duration}"
        self._send_command(command)
        logger.info(f"Movimiento absoluto a: [{x}, {y}, {z}] en {duration}s")
        return True
    
    def move_to_position(self, position: Position, duration: float = 2.0) -> bool:
        """
        Mueve el robot a una posición usando un objeto Position.
        
        Args:
            position: Objeto Position con las coordenadas destino
            duration: Duración del movimiento en segundos
            
        Returns:
            True si el movimiento fue exitoso
        """
        return self.move_to(position.x, position.y, position.z, duration)
    
    # ==================== INFORMACIÓN DEL ROBOT ====================
    
    def get_position(self) -> Position:
        """
        Obtiene la posición actual del robot.
        
        Returns:
            Objeto Position con la posición actual
            
        Example:
            pos = robot.get_position()
            print(f"Robot en: {pos}")
        """
        self._send_command("POS")
        time.sleep(0.1)  # Pequeña pausa para que se actualice el estado
        
        status = self._get_status()
        last_response = status.get('last_response', '')
        
        # Parsear respuesta del tipo "Position: [x,y,z]"
        if 'Position:' in last_response:
            try:
                # Extraer la parte entre corchetes
                start = last_response.find('[')
                end = last_response.find(']')
                if start != -1 and end != -1:
                    coords_str = last_response[start+1:end]
                    coords = [int(x.strip()) for x in coords_str.split(',')]
                    return Position(coords[0], coords[1], coords[2])
            except (ValueError, IndexError) as e:
                logger.warning(f"Error parseando posición: {e}")
        
        # Si no se puede parsear, retornar posición por defecto
        logger.warning("No se pudo obtener la posición, retornando [0,0,0]")
        return Position(0, 0, 0)
    
    def get_limits(self) -> MotorLimits:
        """
        Obtiene los límites de movimiento de cada motor.
        
        Returns:
            Objeto MotorLimits con los límites mínimos y máximos
            
        Example:
            limits = robot.get_limits()
            print(f"Límites: {limits}")
        """
        self._send_command("LIMITS")
        time.sleep(0.1)
        
        status = self._get_status()
        last_response = status.get('last_response', '')
        
        # Parsear respuesta del tipo "Limits: M0[min,max] M1[min,max] M2[min,max]"
        try:
            min_limits = [0, 0, 0]
            max_limits = [3200, 2100, 700]  # Valores por defecto
            
            if 'Limits:' in last_response:
                # Buscar patrones M0[min,max], M1[min,max], M2[min,max]
                import re
                pattern = r'M(\d+)\[(\d+),(\d+)\]'
                matches = re.findall(pattern, last_response)
                
                for match in matches:
                    motor_idx = int(match[0])
                    min_val = int(match[1])
                    max_val = int(match[2])
                    
                    if motor_idx < 3:
                        min_limits[motor_idx] = min_val
                        max_limits[motor_idx] = max_val
            
            return MotorLimits(min_limits, max_limits)
            
        except Exception as e:
            logger.warning(f"Error parseando límites: {e}")
            return MotorLimits([0, 0, 0], [3200, 2100, 700])
    
    def get_status_info(self) -> str:
        """
        Obtiene información detallada del estado del robot.
        
        Returns:
            String con el estado actual del robot
        """
        self._send_command("STATUS")
        time.sleep(0.1)
        
        status = self._get_status()
        return status.get('last_response', 'No status available')
    
    # ==================== CONTROL DEL ROBOT ====================
    
    def reset_position(self) -> bool:
        """
        Resetea la posición del robot a [0, 0, 0].
        
        Returns:
            True si el reset fue exitoso
            
        Example:
            robot.reset_position()
        """
        self._send_command("RESET")
        logger.info("Posición reseteada a [0, 0, 0]")
        return True
    
    def set_position(self, x: int, y: int, z: int) -> bool:
        """
        Establece manualmente la posición actual del robot sin mover los motores.
        
        Args:
            x: Nueva posición en X
            y: Nueva posición en Y  
            z: Nueva posición en Z
            
        Returns:
            True si se estableció correctamente
            
        Example:
            robot.set_position(500, 300, 100)  # Establecer posición actual
        """
        command = f"SETPOS {x} {y} {z}"
        self._send_command(command)
        logger.info(f"Posición establecida manualmente a: [{x}, {y}, {z}]")
        return True
    
    def emergency_stop(self) -> bool:
        """
        Detiene inmediatamente todos los movimientos del robot.
        
        Returns:
            True si el comando fue enviado
            
        Example:
            robot.emergency_stop()
        """
        self._send_command("STOP")
        logger.warning("STOP de emergencia enviado")
        return True
    
    # ==================== MOVIMIENTOS AVANZADOS ====================
    
    def move_sequence(self, positions: List[Tuple[int, int, int, float]], 
                     wait_between: float = 0.5) -> bool:
        """
        Ejecuta una secuencia de movimientos absolutos.
        
        Args:
            positions: Lista de tuplas (x, y, z, duration)
            wait_between: Tiempo de espera entre movimientos en segundos
            
        Returns:
            True si toda la secuencia fue exitosa
            
        Example:
            sequence = [
                (100, 100, 50, 2.0),   # Ir a [100,100,50] en 2s
                (200, 150, 100, 1.5),  # Ir a [200,150,100] en 1.5s
                (0, 0, 0, 3.0)         # Volver al origen en 3s
            ]
            robot.move_sequence(sequence)
        """
        logger.info(f"Iniciando secuencia de {len(positions)} movimientos")
        
        for i, (x, y, z, duration) in enumerate(positions, 1):
            logger.info(f"Movimiento {i}/{len(positions)}: [{x}, {y}, {z}] en {duration}s")
            self.move_to(x, y, z, duration)
            
            if i < len(positions):  # No esperar después del último movimiento
                time.sleep(wait_between)
        
        logger.info("Secuencia completada")
        return True
    
    def home(self, duration: float = 3.0) -> bool:
        """
        Mueve el robot a la posición de origen [0, 0, 0].
        
        Args:
            duration: Duración del movimiento al origen
            
        Returns:
            True si el movimiento fue exitoso
            
        Example:
            robot.home(2.5)  # Ir al origen en 2.5 segundos
        """
        logger.info("Moviendo a posición de origen")
        return self.move_to(0, 0, 0, duration)
    
    def jog(self, axis: str, steps: int, duration: float = 1.0) -> bool:
        """
        Mueve un solo eje del robot (jog).
        
        Args:
            axis: Eje a mover ('x', 'y', 'z')
            steps: Número de pasos (positivo o negativo)
            duration: Duración del movimiento
            
        Returns:
            True si el movimiento fue exitoso
            
        Example:
            robot.jog('x', 100)    # Mover X +100 pasos
            robot.jog('y', -50)    # Mover Y -50 pasos
        """
        axis = axis.lower()
        x = steps if axis == 'x' else 0
        y = steps if axis == 'y' else 0
        z = steps if axis == 'z' else 0
        
        logger.info(f"Jog en eje {axis.upper()}: {steps} pasos")
        return self.move_relative(x, y, z, duration)
    
    # ==================== UTILIDADES ====================
    
    def wait_for_movement(self, check_interval: float = 0.5, timeout: float = 30.0) -> bool:
        """
        Espera hasta que el robot termine su movimiento actual.
        
        Args:
            check_interval: Intervalo de verificación en segundos
            timeout: Timeout máximo en segundos
            
        Returns:
            True si el movimiento terminó, False si hubo timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self.get_status_info()
            if "Status: OK" in status:  # El robot no está moviendose
                return True
            time.sleep(check_interval)
        
        logger.warning(f"Timeout esperando fin de movimiento ({timeout}s)")
        return False
    
    def is_position_safe(self, x: int, y: int, z: int) -> bool:
        """
        Verifica si una posición está dentro de los límites del robot.
        
        Args:
            x, y, z: Coordenadas a verificar
            
        Returns:
            True si la posición es segura
        """
        limits = self.get_limits()
        position = Position(x, y, z)
        return limits.is_position_valid(position)
    
    def get_distance_to(self, x: int, y: int, z: int) -> float:
        """
        Calcula la distancia euclidiana desde la posición actual hasta una posición dada.
        
        Args:
            x, y, z: Coordenadas destino
            
        Returns:
            Distancia en pasos
        """
        current = self.get_position()
        dx = x - current.x
        dy = y - current.y
        dz = z - current.z
        return (dx**2 + dy**2 + dz**2)**0.5
    
    def close(self):
        """Cierra la conexión con el robot"""
        self.session.close()
        logger.info("Conexión cerrada")
    
    def __enter__(self):
        """Soporte para context manager"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Soporte para context manager"""
        self.close()


# ==================== FUNCIONES DE UTILIDAD ====================

def create_robot_connection(ip: str, port: int = 80, timeout: float = 10.0) -> RobotAPI:
    """
    Función de conveniencia para crear una conexión al robot.
    
    Args:
        ip: Dirección IP del ESP32
        port: Puerto HTTP
        timeout: Timeout en segundos
        
    Returns:
        Instancia de RobotAPI conectada
        
    Example:
        robot = create_robot_connection("192.168.1.100")
    """
    return RobotAPI(ip, port, timeout)


def discover_robot(network_base: str = "192.168.1", timeout: float = 2.0) -> Optional[str]:
    """
    Intenta descubrir automáticamente la IP del robot en la red local.
    
    Args:
        network_base: Base de la red (ej: "192.168.1")
        timeout: Timeout por IP probada
        
    Returns:
        IP del robot si se encuentra, None si no
        
    Example:
        ip = discover_robot("192.168.100")
        if ip:
            robot = create_robot_connection(ip)
    """
    logger.info(f"Buscando robot en red {network_base}.x...")
    
    for i in range(1, 255):
        ip = f"{network_base}.{i}"
        try:
            response = requests.get(f"http://{ip}/api/status", timeout=timeout)
            if response.status_code == 200:
                logger.info(f"Robot encontrado en: {ip}")
                return ip
        except requests.exceptions.RequestException:
            continue
    
    logger.warning("No se encontró robot en la red")
    return None


if __name__ == "__main__":
    # Ejemplo de uso
    print("Robot Motor Control API - Ejemplo de uso")
    print("=" * 50)
    
    # Descubrir robot automáticamente o usar IP fija
    robot_ip = discover_robot("192.168.100") or "192.168.100.72"
    
    try:
        # Crear conexión usando context manager
        with create_robot_connection(robot_ip) as robot:
            print(f"Conectado al robot en {robot_ip}")
            
            # Obtener información inicial
            pos = robot.get_position()
            limits = robot.get_limits()
            status = robot.get_status_info()
            
            print(f"Posición actual: {pos}")
            print(f"Límites: {limits}")
            print(f"Estado: {status}")
            
            # Ejemplo de movimientos
            print("\nEjecutando movimientos de ejemplo...")
            
            # Movimiento relativo
            robot.move_relative(50, 100, 25, 1.5)
            time.sleep(2)
            
            # Obtener nueva posición
            new_pos = robot.get_position()
            print(f"Nueva posición: {new_pos}")
            
            # Ir a una posición específica
            robot.move_to(500, 300, 150, 2.0)
            time.sleep(3)
            
            # Secuencia de movimientos
            sequence = [
                (100, 100, 50, 1.0),
                (200, 200, 100, 1.0),
                (300, 100, 75, 1.0)
            ]
            robot.move_sequence(sequence, wait_between=0.5)
            
            # Volver al origen
            robot.home(2.0)
            
            print("Ejemplo completado exitosamente!")
            
    except RobotConnectionError as e:
        print(f"Error de conexión: {e}")
    except RobotCommandError as e:
        print(f"Error de comando: {e}")
    except Exception as e:
        print(f"Error inesperado: {e}")
