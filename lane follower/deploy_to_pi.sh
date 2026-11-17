#!/bin/bash
# =============================================================
# Deploy Line Follower code to Raspberry Pi via SSH + SCP
# Usage:  ./deploy_to_pi.sh [pi-user@pi-ip]
# Default: pi@raspberrypi.local
# =============================================================

PI_TARGET="${1:-pi@raspberrypi.local}"
REMOTE_DIR="~/lane_follower"

echo "============================================="
echo "  Deploying Line Follower to: $PI_TARGET     "
echo "  Remote directory: $REMOTE_DIR              "
echo "============================================="

# Create remote directory
echo "[1/3] Creating remote workspace..."
ssh "$PI_TARGET" "mkdir -p $REMOTE_DIR" || { echo "SSH failed! Is SSH enabled on Pi?"; exit 1; }

# Copy all Python files and config
echo "[2/3] Copying project files..."
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
scp "$SCRIPT_DIR"/{config.py,motor_driver.py,pid_controller.py,line_detector.py,line_follower.py,web_ui.py,calibrate_color.py,test_motors.py,requirements.txt,pi_setup.sh} \
    "$PI_TARGET:$REMOTE_DIR/"

# Install dependencies on Pi
echo "[3/3] Installing dependencies on Pi..."
ssh "$PI_TARGET" "cd $REMOTE_DIR && pip3 install flask 2>/dev/null || sudo apt-get install -y python3-flask"

echo ""
echo "============================================="
echo "  DEPLOY COMPLETE!"
echo "============================================="
echo ""
echo "  To start the Web UI on the Pi, SSH in and run:"
echo "    ssh $PI_TARGET"
echo "    cd $REMOTE_DIR"
echo "    python3 web_ui.py"
echo ""
echo "  Then open in browser:"
echo "    http://raspberrypi.local:5000"
echo "============================================="
