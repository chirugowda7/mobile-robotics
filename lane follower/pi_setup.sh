#!/bin/bash
# =============================================================
# Raspberry Pi Setup Script for Line Follower Robot
# Run this ONCE on the Pi directly (with monitor+keyboard)
# or copy to the Pi's SD card boot partition.
# =============================================================

set -e
echo "============================================="
echo "  Line Follower - Raspberry Pi Setup Script  "
echo "============================================="

# 1. Enable and Start SSH
echo "[1/5] Enabling SSH service..."
sudo systemctl enable ssh
sudo systemctl start ssh
echo "      SSH is now ENABLED and RUNNING."

# 2. Install Required Packages
echo "[2/4] Installing required packages..."
sudo apt-get update -y
sudo apt-get install -y python3-opencv python3-numpy python3-pip python3-flask openssh-server v4l-utils

# 3. Install GPIO libraries (supports Raspberry Pi OS Bookworm & legacy)
echo "[3/4] Installing GPIO libraries..."
sudo apt-get install -y python3-rpi-lgpio 2>/dev/null || sudo apt-get install -y python3-rpi.gpio 2>/dev/null || true

# 4. Create workspace directory
echo "[4/4] Ensuring workspace directory exists..."
mkdir -p ~/lane_follower
echo "      Workspace: ~/lane_follower"

# Print connection info
echo ""
echo "============================================="
echo "  SETUP COMPLETE!"
echo "============================================="
echo "  SSH is now enabled."
echo "  WiFi: ROBO-AI (configured)"
echo ""
echo "  Pi IP address(es):"
hostname -I
echo ""
echo "  From your laptop, connect with:"
echo "    ssh pi@raspberrypi.local"
echo "    or"
echo "    ssh pi@$(hostname -I | awk '{print $1}')"
echo "============================================="
