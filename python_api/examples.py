#!/usr/bin/env python3
"""
Ejemplos de uso de la Robot API
==============================

Este archivo contiene ejemplos prácticos de cómo usar la API del robot
para diferentes tipos de operaciones y patrones de movimiento.
"""

import time
import math
from robot_api import create_robot_connection, Position, RobotConnectionError, RobotCommandError


def basic_movements_example(robot_ip: str):
    """Ejemplo básico de movimientos del robot"""
    print("=== Ejemplo: Movimientos Básicos ===")
    
    with create_robot_connection(robot_ip) as robot:
        # Obtener información inicial
        print(f"Posición inicial: {robot.get_position()}")
        print(f"Límites: {robot.get_limits()}")
        
        # Movimientos relativos simples
        print("\n1. Movimientos relativos...")
        robot.move_relative(100, 0, 0, 1.0)    # X +100
        time.sleep(1.5)
        
        robot.move_relative(0, 150, 0, 1.0)    # Y +150
        time.sleep(1.5)
        
        robot.move_relative(0, 0, 50, 1.0)     # Z +50
        time.sleep(1.5)
        
        print(f"Posición después de movimientos: {robot.get_position()}")
        
        # Movimiento absoluto
        print("\n2. Movimiento absoluto...")
        robot.move_to(500, 400, 200, 2.0)
        time.sleep(2.5)
        
        print(f"Posición final: {robot.get_position()}")
        
        # Volver al origen
        print("\n3. Volver al origen...")
        robot.home(3.0)
        time.sleep(3.5)
        
        print(f"Posición en origen: {robot.get_position()}")


def jog_movements_example(robot_ip: str):
    """Ejemplo de movimientos de jog (un eje a la vez)"""
    print("=== Ejemplo: Movimientos de Jog ===")
    
    with create_robot_connection(robot_ip) as robot:
        robot.reset_position()
        
        print("Movimientos de jog en cada eje...")
        
        # Jog en X
        for i in range(3):
            print(f"Jog X +50 (paso {i+1})")
            robot.jog('x', 50, 0.8)
            time.sleep(1)
        
        # Jog en Y
        for i in range(3):
            print(f"Jog Y +75 (paso {i+1})")
            robot.jog('y', 75, 0.8)
            time.sleep(1)
        
        # Jog en Z
        for i in range(2):
            print(f"Jog Z +25 (paso {i+1})")
            robot.jog('z', 25, 0.8)
            time.sleep(1)
        
        print(f"Posición final: {robot.get_position()}")
        
        # Regresar con jog negativo
        print("\nRegresando con jog negativo...")
        robot.jog('z', -50, 1.0)
        time.sleep(1.2)
        robot.jog('y', -225, 1.5)
        time.sleep(2)
        robot.jog('x', -150, 1.5)
        time.sleep(2)
        
        print(f"Posición final: {robot.get_position()}")


def pattern_movements_example(robot_ip: str):
    """Ejemplo de movimientos en patrones geométricos"""
    print("=== Ejemplo: Patrones de Movimiento ===")
    
    with create_robot_connection(robot_ip) as robot:
        robot.reset_position()
        
        # Patrón: Cuadrado en el plano XY
        print("\n1. Dibujando cuadrado en plano XY...")
        square_size = 200
        square_points = [
            (0, 0, 0, 1.0),
            (square_size, 0, 0, 1.5),
            (square_size, square_size, 0, 1.5),
            (0, square_size, 0, 1.5),
            (0, 0, 0, 1.5)
        ]
        robot.move_sequence(square_points, wait_between=0.5)
        time.sleep(1)
        
        # Patrón: Movimiento helicoidal
        print("\n2. Movimiento helicoidal...")
        center_x, center_y = 400, 300
        radius = 150
        height_step = 15
        
        helix_points = []
        for i in range(8):
            angle = i * (2 * math.pi / 8)
            x = center_x + int(radius * math.cos(angle))
            y = center_y + int(radius * math.sin(angle))
            z = i * height_step
            helix_points.append((x, y, z, 1.0))
        
        robot.move_sequence(helix_points, wait_between=0.3)
        time.sleep(1)
        
        # Volver al origen
        robot.home(3.0)


def precision_movements_example(robot_ip: str):
    """Ejemplo de movimientos de precisión y verificación"""
    print("=== Ejemplo: Movimientos de Precisión ===")
    
    with create_robot_connection(robot_ip) as robot:
        robot.reset_position()
        
        # Movimientos micro
        print("\n1. Movimientos de micro-posicionamiento...")
        for i in range(10):
            robot.move_relative(5, 3, 1, 0.3)  # Movimientos muy pequeños
            time.sleep(0.5)
            pos = robot.get_position()
            print(f"Micro-paso {i+1}: {pos}")
        
        # Verificación de límites
        print("\n2. Prueba de límites...")
        limits = robot.get_limits()
        
        # Intentar ir cerca del límite máximo
        safe_pos = Position(
            limits.max_limits[0] - 100,
            limits.max_limits[1] - 100,
            limits.max_limits[2] - 50
        )
        
        if robot.is_position_safe(safe_pos.x, safe_pos.y, safe_pos.z):
            print(f"Moviendo a posición cerca del límite: {safe_pos}")
            robot.move_to_position(safe_pos, 3.0)
            time.sleep(3.5)
        else:
            print("La posición no es segura")
        
        # Calcular distancia
        distance = robot.get_distance_to(0, 0, 0)
        print(f"Distancia al origen: {distance:.2f} pasos")
        
        robot.home(4.0)


def interactive_control_example(robot_ip: str):
    """Ejemplo de control interactivo por consola"""
    print("=== Ejemplo: Control Interactivo ===")
    print("Comandos disponibles:")
    print("  w/s: Mover Y +/-")
    print("  a/d: Mover X -/+") 
    print("  q/e: Mover Z +/-")
    print("  h: Ir al origen")
    print("  p: Mostrar posición")
    print("  r: Reset posición")
    print("  x: Salir")
    
    step_size = 50
    move_duration = 0.8
    
    with create_robot_connection(robot_ip) as robot:
        print(f"\nPosición inicial: {robot.get_position()}")
        print(f"Tamaño de paso: {step_size}")
        
        while True:
            try:
                cmd = input("\nComando: ").lower().strip()
                
                if cmd == 'x':
                    break
                elif cmd == 'w':
                    robot.jog('y', step_size, move_duration)
                elif cmd == 's':
                    robot.jog('y', -step_size, move_duration)
                elif cmd == 'a':
                    robot.jog('x', -step_size, move_duration)
                elif cmd == 'd':
                    robot.jog('x', step_size, move_duration)
                elif cmd == 'q':
                    robot.jog('z', step_size, move_duration)
                elif cmd == 'e':
                    robot.jog('z', -step_size, move_duration)
                elif cmd == 'h':
                    robot.home(2.0)
                elif cmd == 'p':
                    pos = robot.get_position()
                    print(f"Posición actual: {pos}")
                    continue
                elif cmd == 'r':
                    robot.reset_position()
                    print("Posición reseteada")
                    continue
                else:
                    print("Comando no reconocido")
                    continue
                
                # Mostrar posición después del movimiento
                time.sleep(move_duration + 0.2)
                pos = robot.get_position()
                print(f"Nueva posición: {pos}")
                
            except KeyboardInterrupt:
                print("\nInterrumpido por usuario")
                break
            except Exception as e:
                print(f"Error: {e}")


def stress_test_example(robot_ip: str):
    """Ejemplo de prueba de estrés con muchos movimientos"""
    print("=== Ejemplo: Prueba de Estrés ===")
    
    with create_robot_connection(robot_ip) as robot:
        robot.reset_position()
        
        print("Ejecutando 50 movimientos aleatorios...")
        
        import random
        
        for i in range(50):
            # Generar movimiento aleatorio pequeño
            x = random.randint(-20, 20)
            y = random.randint(-20, 20)
            z = random.randint(-10, 10)
            duration = random.uniform(0.3, 0.8)
            
            try:
                robot.move_relative(x, y, z, duration)
                time.sleep(duration + 0.1)
                
                if i % 10 == 0:
                    pos = robot.get_position()
                    print(f"Movimiento {i+1}/50 - Posición: {pos}")
                    
            except Exception as e:
                print(f"Error en movimiento {i+1}: {e}")
                break
        
        print(f"Posición final: {robot.get_position()}")
        robot.home(2.0)


def main():
    """Función principal para ejecutar ejemplos"""
    # Configurar IP del robot (cambiar según tu red)
    ROBOT_IP = "192.168.100.72"
    
    examples = {
        "1": ("Movimientos Básicos", basic_movements_example),
        "2": ("Movimientos de Jog", jog_movements_example),
        "3": ("Patrones Geométricos", pattern_movements_example),
        "4": ("Movimientos de Precisión", precision_movements_example),
        "5": ("Control Interactivo", interactive_control_example),
        "6": ("Prueba de Estrés", stress_test_example)
    }
    
    print("Robot API - Ejemplos de Uso")
    print("=" * 40)
    print(f"IP del robot: {ROBOT_IP}")
    print("\nEjemplos disponibles:")
    
    for key, (name, _) in examples.items():
        print(f"  {key}: {name}")
    
    print("  0: Salir")
    
    while True:
        try:
            choice = input("\nSelecciona un ejemplo (0-6): ").strip()
            
            if choice == "0":
                break
            elif choice in examples:
                name, func = examples[choice]
                print(f"\n{'='*20} {name} {'='*20}")
                func(ROBOT_IP)
                print(f"{'='*60}")
            else:
                print("Opción no válida")
                
        except KeyboardInterrupt:
            print("\nSaliendo...")
            break
        except RobotConnectionError as e:
            print(f"Error de conexión: {e}")
            print("Verifica que el robot esté encendido y conectado")
        except RobotCommandError as e:
            print(f"Error de comando: {e}")
        except Exception as e:
            print(f"Error inesperado: {e}")


if __name__ == "__main__":
    main()
