"""
Script SIMPLE para enviar la posición del ratón por CAN
Basado en el send.py que YA FUNCIONA
"""

import time
import ctypes

# Usar la API de Windows para obtener la posición del ratón (SIN DEPENDENCIAS EXTRA)
class POINT(ctypes.Structure):
    _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]

def get_mouse_position():
    """Obtiene la posición actual del ratón usando la API de Windows"""
    pt = POINT()
    ctypes.windll.user32.GetCursorPos(ctypes.byref(pt))
    return pt.x, pt.y

def get_screen_size():
    """Obtiene el tamaño de la pantalla usando la API de Windows"""
    user32 = ctypes.windll.user32
    screen_width = user32.GetSystemMetrics(0)  # SM_CXSCREEN
    screen_height = user32.GetSystemMetrics(1)  # SM_CYSCREEN
    return screen_width, screen_height

# ========== AHORA USA PYTHON-CAN (MÁS SIMPLE) ==========
try:
    import can
    print("✅ Módulo 'can' encontrado")
except ImportError:
    print("❌ Instala python-can: pip install python-can")
    print("   Luego ejecuta de nuevo este script")
    exit(1)

def main():
    print("🚀 Iniciando envío de posición del ratón por CAN...")
    
    # Obtener tamaño de pantalla
    try:
        screen_width, screen_height = get_screen_size()
        print(f"📺 Tamaño de pantalla: {screen_width}x{screen_height}")
    except:
        screen_width, screen_height = 1920, 1080
        print(f"📺 Usando tamaño por defecto: {screen_width}x{screen_height}")
    
    # Conectar al bus CAN
    # Nota: Ajusta el channel según tu dispositivo
    # Para INNOMAKER USB2CAN en Windows, prueba 'COM3', 'COM4', etc.
    # O usa 'virtual' para pruebas sin hardware
    try:
        # Intenta con interfaz virtual primero (para pruebas)
        bus = can.interface.Bus(channel='test', bustype='virtual', bitrate=500000)
        print("✅ Conectado al bus CAN VIRTUAL (modo prueba)")
        print("   Para conectar al hardware real, modifica el 'channel' y 'bustype'")
    except Exception as e:
        print(f"❌ Error al conectar al bus CAN: {e}")
        print("\n💡 Para usar el hardware real:")
        print("   1. Averigua qué 'bustype' necesitas (socketcan, pcan, ixxat, etc.)")
        print("   2. Modifica la línea 'bus = can.interface.Bus(...)' en el código")
        return
    
    print("\n🎮 Moviendo el ratón enviará su posición por CAN...")
    print("   Presiona Ctrl+C para detener\n")
    
    frame_count = 0
    last_x, last_y = -1, -1
    
    try:
        while True:
            # Obtener posición del ratón
            x, y = get_mouse_position()
            
            # Solo enviar si cambió la posición
            if x != last_x or y != last_y:
                # Normalizar a 16 bits (0-65535)
                x_norm = int((x / screen_width) * 65535)
                y_norm = int((y / screen_height) * 65535)
                
                # Crear mensaje CAN
                # ID: 0x100 (PC1 - Jugador 1)
                # Datos: [X_high, X_low, Y_high, Y_low, 0, 0, 0, 0]
                data = [
                    (x_norm >> 8) & 0xFF,   # X byte alto
                    x_norm & 0xFF,          # X byte bajo
                    (y_norm >> 8) & 0xFF,   # Y byte alto
                    y_norm & 0xFF,          # Y byte bajo
                    0, 0, 0, 0              # Bytes extra (sin usar)
                ]
                
                msg = can.Message(
                    arbitration_id=0x100,
                    data=data,
                    is_extended_id=False
                )
                
                # Enviar mensaje
                try:
                    bus.send(msg)
                    frame_count += 1
                    print(f"📤 #{frame_count}: Pos({x:4d},{y:4d}) -> Norm({x_norm:5d},{y_norm:5d}) | ID: 0x{msg.arbitration_id:03X}")
                    last_x, last_y = x, y
                except can.CanError as e:
                    print(f"⚠️  Error enviando: {e}")
            
            # Delay de 40ms (25 fps)
            time.sleep(0.04)
            
    except KeyboardInterrupt:
        print("\n\n⏹️  Deteniendo...")
    finally:
        bus.shutdown()
        print("✅ Bus CAN cerrado correctamente")

if __name__ == "__main__":
    main()
