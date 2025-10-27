

import os
import can
import time
import sys
import select
import tty
import termios
import threading

# Check system name
print(f"Operating System: {os.name}")

# For macOS with Innomaker USB2CAN adapter
# The Innomaker adapter uses the gs_usb protocol (Geschwister Schneider USB/CAN)
print("Looking for USB2CAN adapter...")
print("Configuring CAN interface at 500 kbps...")

# Try to use gs_usb interface (for Innomaker USB2CAN)
try:
    can1 = can.interface.Bus(
        interface='gs_usb',
        channel=0,  # Channel number (not string)
        bitrate=500000
    )
    print("✓ CAN bus initialized successfully with gs_usb (USB2CAN adapter)!")
except Exception as e:
    print(f"✗ gs_usb interface failed: {e}")
    print("\nTroubleshooting:")
    print("1. Make sure the Innomaker USB2CAN adapter is plugged in (not the ESP32)")
    print("2. Close the Innomaker application if it's running")
    print("3. Try unplugging and replugging the USB2CAN adapter")
    print("4. If the problem persists, try: sudo python3 send.py")
    sys.exit(1)

# Message definitions for WASD commands
messages = {
    'w': can.Message(arbitration_id=0x100, data=[0x57, 0, 0, 0, 0, 0, 0, 0]),  # 'W' in hex
    'a': can.Message(arbitration_id=0x101, data=[0x41, 0, 0, 0, 0, 0, 0, 0]),  # 'A' in hex
    's': can.Message(arbitration_id=0x102, data=[0x53, 0, 0, 0, 0, 0, 0, 0]),  # 'S' in hex
    'd': can.Message(arbitration_id=0x103, data=[0x44, 0, 0, 0, 0, 0, 0, 0]),  # 'D' in hex
}

# Flag to control the receiver thread
running = True

def receive_messages():
    """Background thread to receive CAN messages"""
    global running
    print("[Receiver] Starting receive thread...\n")
    
    while running:
        try:
            # Wait for a message with a timeout
            msg = can1.recv(timeout=0.5)
            
            if msg is not None:
                # Display received message
                data_hex = ' '.join([f'{b:02X}' for b in msg.data])
                timestamp = time.strftime('%H:%M:%S')
                print(f"\r[{timestamp}] ← Received - ID: 0x{msg.arbitration_id:03X}, Data: [{data_hex}]")
                print("Press W/A/S/D to send, Q to quit: ", end='', flush=True)
                
        except Exception as e:
            if running:  # Only print errors if we're still running
                print(f"\r[Receiver] Error: {e}")
    
    print("[Receiver] Thread stopped")

def get_key():
    """Get a single keypress without requiring Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        # Check if input is available
        if select.select([sys.stdin], [], [], 0.1)[0]:
            ch = sys.stdin.read(1)
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

print("\n=== CAN Bus WASD Controller ===")
print("Bitrate: 500 kbps")
print("Press W, A, S, D to send messages")
print("Press Q to quit\n")

# Start the receiver thread
receiver_thread = threading.Thread(target=receive_messages, daemon=True)
receiver_thread.start()

# Give the receiver thread a moment to start
time.sleep(0.5)

print("Press W/A/S/D to send, Q to quit: ", end='', flush=True)

try:
    while True:
        key = get_key()
        
        if key:
            key_lower = key.lower()
            
            if key_lower == 'q':
                print("\n\nQuitting...")
                running = False
                break
            
            if key_lower in messages:
                msg = messages[key_lower]
                can1.send(msg)
                timestamp = time.strftime('%H:%M:%S')
                data_hex = ' '.join([f'{b:02X}' for b in msg.data])
                print(f"\r[{timestamp}] → Sent '{key_lower.upper()}' - ID: 0x{msg.arbitration_id:03X}, Data: [{data_hex}]")
                print("Press W/A/S/D to send, Q to quit: ", end='', flush=True)
        
        time.sleep(0.01)  # Small delay to avoid CPU spinning

except KeyboardInterrupt:
    print("\n\nInterrupted by user")
    running = False
finally:
    running = False
    receiver_thread.join(timeout=1.0)
    can1.shutdown()
    print("CAN interface closed")

