#!/usr/bin/env python3
"""Test script to diagnose USB2CAN connection issues"""

import usb.core
import usb.util

print("Searching for USB devices...")
print()

# Find all USB devices
devices = usb.core.find(find_all=True)

print("All USB devices:")
for device in devices:
    try:
        print(f"  ID {device.idVendor:04x}:{device.idProduct:04x}")
        try:
            print(f"    Manufacturer: {device.manufacturer}")
            print(f"    Product: {device.product}")
        except:
            print("    (Cannot read device info - may need permissions)")
    except Exception as e:
        print(f"    Error: {e}")

print("\n" + "="*50)
print("Looking specifically for GS_USB device (Innomaker)...")
print("  Vendor ID: 0x1d50")
print("  Product ID: 0x606f")
print()

# Find GS_USB device
dev = usb.core.find(idVendor=0x1d50, idProduct=0x606f)

if dev is None:
    print("❌ GS_USB device NOT found!")
    print("   Make sure the Innomaker USB2CAN is plugged in")
else:
    print("✓ GS_USB device FOUND!")
    print(f"  Bus: {dev.bus}")
    print(f"  Address: {dev.address}")
    
    # Try to access it
    try:
        print("\nTrying to access device...")
        # Check if kernel driver is attached
        if dev.is_kernel_driver_active(0):
            print("  Kernel driver is active - this might cause issues")
            print("  Attempting to detach...")
            try:
                dev.detach_kernel_driver(0)
                print("  ✓ Kernel driver detached")
            except Exception as e:
                print(f"  ✗ Could not detach: {e}")
        else:
            print("  No kernel driver attached")
            
        # Try to set configuration
        try:
            dev.set_configuration()
            print("  ✓ Configuration set successfully")
        except Exception as e:
            print(f"  ✗ Could not set configuration: {e}")
            
    except usb.core.USBError as e:
        print(f"  ✗ USB Error: {e}")
        print("\nThis might be a permissions issue.")
        print("Try running with: sudo python3 test_usb.py")
