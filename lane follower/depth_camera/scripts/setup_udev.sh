#!/bin/bash
set -e

echo "=== Orbbec Gemini E Depth Camera - USB Permissions Setup ==="

RULE_FILE="/etc/udev/rules.d/99-obsensor-libusb.rules"

echo "Creating udev rule for Orbbec cameras (Vendor ID: 2bc5)..."

RULE_CONTENT='# Orbbec USB device udev rules
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="video", ATTRS{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
'

if [ "$EUID" -ne 0 ]; then
    echo "Writing udev rule via sudo..."
    echo "$RULE_CONTENT" | sudo tee "$RULE_FILE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
else
    echo "$RULE_CONTENT" > "$RULE_FILE"
    udevadm control --reload-rules
    udevadm trigger
fi

echo "[SUCCESS] Orbbec udev rules configured successfully at $RULE_FILE"
echo "Please unplug and re-plug your Orbbec depth camera USB cable."
