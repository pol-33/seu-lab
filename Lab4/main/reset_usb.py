#!/usr/bin/env python3
"""Reset the USB2CAN device"""

import usb.core
import usb.util
import time

print("Resetting USB2CAN device...")

# Find GS_USB device
dev = usb.core.find(idVendor=0x1d50, idProduct=0x606f)

if dev is None:
    print("✗ USB2CAN device not found!")
else:
    print("✓ Found USB2CAN device")
    try:
        # Reset the device
        dev.reset()
        print("✓ Device reset successful")
        time.sleep(2)
        print("Device should be ready now")
    except Exception as e:
        print(f"✗ Reset failed: {e}")
        print("Try unplugging and replugging the device manually")
