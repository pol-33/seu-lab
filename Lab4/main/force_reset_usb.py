#!/usr/bin/env python3
"""Force reset and reclaim USB2CAN device"""

import usb.core
import usb.util
import time
import sys

print("=== USB2CAN Force Reset ===\n")

# Find GS_USB device
print("1. Searching for device...")
dev = usb.core.find(idVendor=0x1d50, idProduct=0x606f)

if dev is None:
    print("✗ USB2CAN device not found!")
    print("   Make sure it's plugged in")
    sys.exit(1)

print(f"✓ Found device: {dev}")
print(f"   Manufacturer: {usb.util.get_string(dev, dev.iManufacturer)}")
print(f"   Product: {usb.util.get_string(dev, dev.iProduct)}")
print(f"   Serial: {usb.util.get_string(dev, dev.iSerialNumber)}")

# Detach kernel driver if active
print("\n2. Checking for kernel drivers...")
for cfg in dev:
    for intf in cfg:
        if dev.is_kernel_driver_active(intf.bInterfaceNumber):
            print(f"   Detaching kernel driver from interface {intf.bInterfaceNumber}")
            try:
                dev.detach_kernel_driver(intf.bInterfaceNumber)
            except usb.core.USBError as e:
                print(f"   Could not detach kernel driver: {e}")

# Reset device
print("\n3. Resetting device...")
try:
    dev.reset()
    print("✓ Device reset successful")
except Exception as e:
    print(f"✗ Reset failed: {e}")
    sys.exit(1)

print("\n4. Waiting for device to re-enumerate...")
time.sleep(3)

# Try to find it again
print("\n5. Verifying device is back...")
dev = usb.core.find(idVendor=0x1d50, idProduct=0x606f)
if dev is None:
    print("✗ Device not found after reset")
    print("   Try unplugging and replugging")
    sys.exit(1)

print("✓ Device found and ready!")
print("\n=== Reset Complete ===")
print("You can now run: sudo .venv/bin/python3 send.py")
