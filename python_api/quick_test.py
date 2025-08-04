#!/usr/bin/env python3
"""
Prueba rápida de la Robot API
============================

Script simple para verificar que la conexión y comandos básicos funcionen.
"""

import sys
from robot_api import create_robot_connection, discover_robot, RobotConnectionError


def quick_test(robot_ip=None):
    """Prueba rápida de conectividad y comandos básicos"""
    
    # Descubrir robot si no se proporciona IP
    if not robot_ip:
        print("Buscando robot en la red...")
        robot_ip = discover_robot("192.168.100")
        if not robot_ip:
            robot_ip = discover_robot("192.168.1")
        if not robot_ip:
            print("❌ No se encontró robot en la red")
            return False
    
    try:
        print(f"🔌 Conectando a robot en {robot_ip}...")
        
        with create_robot_connection(robot_ip) as robot:
            print("✅ Conexión exitosa!")
            
            # Pruebas básicas
            print("\n📍 Obteniendo información del robot...")
            
            pos = robot.get_position()
            print(f"   Posición actual: {pos}")
            
            limits = robot.get_limits()
            print(f"   Límites: {limits}")
            
            status = robot.get_status_info()
            print(f"   Estado: {status}")
            
            # Prueba de movimiento pequeño
            print("\n🏃 Probando movimiento pequeño...")
            original_pos = robot.get_position()
            
            robot.move_relative(10, 10, 5, 1.0)
            print("   Moviendo [+10, +10, +5]...")
            
            import time
            time.sleep(1.5)
            
            new_pos = robot.get_position()
            print(f"   Nueva posición: {new_pos}")
            
            # Regresar a posición original
            robot.move_to(original_pos.x, original_pos.y, original_pos.z, 1.0)
            print("   Regresando a posición original...")
            
            time.sleep(1.5)
            final_pos = robot.get_position()
            print(f"   Posición final: {final_pos}")
            
            print("\n✅ Todas las pruebas completadas exitosamente!")
            return True
            
    except RobotConnectionError as e:
        print(f"❌ Error de conexión: {e}")
        return False
    except Exception as e:
        print(f"❌ Error inesperado: {e}")
        return False


def main():
    """Función principal"""
    print("Robot API - Prueba Rápida")
    print("=" * 40)
    
    # Permitir IP como argumento de línea de comandos
    robot_ip = None
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
        print(f"Usando IP proporcionada: {robot_ip}")
    
    success = quick_test(robot_ip)
    
    if success:
        print("\n🎉 El robot está funcionando correctamente!")
        print("\nPuedes usar ahora:")
        print("  - robot_api.py para programar movimientos")
        print("  - examples.py para ver ejemplos avanzados")
    else:
        print("\n❌ Hubo problemas con la conexión o funcionamiento")
        print("\nVerifica:")
        print("  - Que el ESP32 esté encendido")
        print("  - Que esté conectado al WiFi")
        print("  - Que la IP sea correcta")


if __name__ == "__main__":
    main()
