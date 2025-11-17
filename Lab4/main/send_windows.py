import os
import can
import time
import sys
import threading
import msvcrt  # Windows-specific module for keyboard input

# Check system name
print(f"Operating System: {os.name}")

# Select Player (Cat or Mouse)
print("\n=== JOC DEL GAT I LA RATA ===")
print("Tria el teu jugador:")
print("1 - Gat (Cat) - Jugador 1")
print("2 - Rata (Mouse) - Jugador 2")

while True:
    choice = input("Escull (1 o 2): ").strip()
    if choice == '1':
        player = "Cat"
        base_id = 0x100
        break
    elif choice == '2':
        player = "Mouse"
        base_id = 0x200
        break
    else:
        print("Opció invàlida. Tria 1 o 2.")

print(f"\n✓ Has escollit: {player} (Base ID: 0x{base_id:03X})")

# For Windows with Innomaker USB2CAN adapter
print("\nLooking for USB2CAN adapter...")
print("Configuring CAN interface at 500 kbps...")

# Try to use gs_usb interface (for Innomaker USB2CAN)
try:
    can1 = can.interface.Bus(
        interface='gs_usb',
        channel=0,
        bitrate=500000
    )
    print("✓ CAN bus initialized successfully with gs_usb (USB2CAN adapter)!")
except Exception as e:
    print(f"✗ gs_usb interface failed: {e}")
    print("\nTroubleshooting:")
    print("1. Make sure the Innomaker USB2CAN adapter is plugged in")
    print("2. Close the Innomaker application if it's running")
    print("3. Try unplugging and replugging the USB2CAN adapter")
    print("4. Install libusb: Download from https://libusb.info/")
    print("5. Install pyusb: pip install pyusb")
    sys.exit(1)

# Message definitions for WASD commands with selected player base ID
messages = {
    'w': can.Message(arbitration_id=base_id + 0, data=[0x57, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),  # W - Up
    'a': can.Message(arbitration_id=base_id + 1, data=[0x41, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),  # A - Left
    's': can.Message(arbitration_id=base_id + 2, data=[0x53, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),  # S - Down
    'd': can.Message(arbitration_id=base_id + 3, data=[0x44, 0, 0, 0, 0, 0, 0, 0], is_extended_id=False),  # D - Right
}

# Flag to control the receiver thread
running = True

def receive_messages():
    """Background thread to receive CAN messages"""
    global running
    print("[Receiver] Starting receive thread...\n")
    
    while running:
        try:
            msg = can1.recv(timeout=0.5)
            
            if msg is not None:
                data_hex = ' '.join([f'{b:02X}' for b in msg.data])
                timestamp = time.strftime('%H:%M:%S')
                print(f"\r[{timestamp}] ← CAN RX - ID: 0x{msg.arbitration_id:03X}, Data: [{data_hex}]")
                print(f"[{player}] Press W/A/S/D to move, Q to quit: ", end='', flush=True)
                
        except Exception as e:
            if running:
                print(f"\r[Receiver] Error: {e}")
    
    print("[Receiver] Thread stopped")

def get_key():
    """Get a single keypress without requiring Enter (Windows version)"""
    if msvcrt.kbhit():
        ch = msvcrt.getch()
        # Handle special characters
        try:
            return ch.decode('utf-8').lower()
        except:
            return None
    return None

print("\n=== CAN Bus Game Controller ===")
print(f"Player: {player}")
print(f"CAN IDs: 0x{base_id:03X}-0x{base_id+3:03X}")
print("Bitrate: 500 kbps")
print("\nControls:")
print("  W - Move Up")
print("  A - Move Left")
print("  S - Move Down")
print("  D - Move Right")
print("  Q - Quit\n")

# Start the receiver thread
receiver_thread = threading.Thread(target=receive_messages, daemon=True)
receiver_thread.start()
time.sleep(0.5)

print(f"[{player}] Press W/A/S/D to move, Q to quit: ", end='', flush=True)

try:
    while True:
        key = get_key()
        
        if key:
            if key == 'q':
                print("\n\nQuitting...")
                running = False
                break
            
            if key in messages:
                msg = messages[key]
                can1.send(msg)
                timestamp = time.strftime('%H:%M:%S')
                data_hex = ' '.join([f'{b:02X}' for b in msg.data])
                direction = {'w': 'UP', 'a': 'LEFT', 's': 'DOWN', 'd': 'RIGHT'}[key]
                print(f"\r[{timestamp}] → [{player}] {direction} - ID: 0x{msg.arbitration_id:03X}, Data: [{data_hex}]")
                print(f"[{player}] Press W/A/S/D to move, Q to quit: ", end='', flush=True)
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n\nInterrupted by user")
    running = False
finally:
    running = False
    receiver_thread.join(timeout=1.0)
    can1.shutdown()
    print("CAN interface closed")
    print(f"\nGame Over! Thanks for playing as {player}!")
