#!/usr/bin/env python3
"""Test if we can access the CAN device"""

import can
import sys

print("Testing CAN interface access...")
print("Attempting to open gs_usb interface...")

try:
    bus = can.interface.Bus(
        interface='gs_usb',
        channel=0,
        bitrate=500000
    )
    print("✓ SUCCESS! CAN interface opened successfully!")
    print(f"  Interface: {bus}")
    bus.shutdown()
    print("✓ Interface closed cleanly")
    sys.exit(0)
    
except Exception as e:
    print(f"✗ FAILED: {e}")
    print(f"  Error type: {type(e).__name__}")
    print("\nTroubleshooting:")
    print("1. Close the Innomaker GUI application if it's running")
    print("2. Make sure you're running with sudo")
    print("3. Try: sudo python3 reset_usb.py")
    print("4. Then try again")
    sys.exit(1)
