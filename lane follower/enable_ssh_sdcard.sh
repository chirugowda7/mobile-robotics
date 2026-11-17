#!/bin/bash
# =============================================================
# Enable SSH on Raspberry Pi by writing to its SD card boot
# partition. Run this on your LAPTOP after inserting the Pi's
# SD card.
# =============================================================

echo "============================================="
echo "  Enable SSH on Raspberry Pi SD Card         "
echo "============================================="

# Find the boot partition (common mount points)
BOOT_DIR=""
for candidate in \
    /media/$USER/bootfs \
    /media/$USER/boot \
    /media/bootfs \
    /media/boot \
    /mnt/bootfs \
    /mnt/boot; do
    if [ -d "$candidate" ]; then
        BOOT_DIR="$candidate"
        break
    fi
done

if [ -z "$BOOT_DIR" ]; then
    echo "[ERROR] Could not find Pi boot partition!"
    echo "        Insert the Pi's SD card and check:"
    echo "        ls /media/$USER/"
    echo ""
    echo "        Or specify manually:"
    echo "        sudo touch /media/$USER/<boot-partition-name>/ssh"
    exit 1
fi

echo "[FOUND] Boot partition: $BOOT_DIR"

# 1. Create empty 'ssh' file to enable SSH on boot
echo "[1/3] Enabling SSH..."
sudo touch "$BOOT_DIR/ssh"
echo "       Created: $BOOT_DIR/ssh"

# 2. Configure WiFi (ROBO-AI) via wpa_supplicant
echo "[2/3] Configuring WiFi network ROBO-AI..."

# For Raspberry Pi OS Bookworm (Pi 5 / newer) - uses NetworkManager
# Create NetworkManager connection file on rootfs
ROOTFS=""
for candidate in \
    /media/$USER/rootfs \
    /media/rootfs \
    /mnt/rootfs; do
    if [ -d "$candidate/etc" ]; then
        ROOTFS="$candidate"
        break
    fi
done

# Try wpa_supplicant method (works on Bullseye and older)
WPA_CONF="$BOOT_DIR/wpa_supplicant.conf"
cat > /tmp/wpa_supplicant.conf << 'WPAEOF'
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=IN

network={
    ssid="ROBO-AI"
    psk="123456789"
    key_mgmt=WPA-PSK
}
WPAEOF
sudo cp /tmp/wpa_supplicant.conf "$WPA_CONF"
echo "       Created: $WPA_CONF"

# Also write NetworkManager config for Bookworm
if [ -n "$ROOTFS" ] && [ -d "$ROOTFS/etc/NetworkManager" ]; then
    echo "       (Bookworm detected - also writing NetworkManager config)"
    sudo mkdir -p "$ROOTFS/etc/NetworkManager/system-connections"
    sudo tee "$ROOTFS/etc/NetworkManager/system-connections/ROBO-AI.nmconnection" > /dev/null << 'NMEOF'
[connection]
id=ROBO-AI
type=wifi
autoconnect=true

[wifi]
ssid=ROBO-AI
mode=infrastructure

[wifi-security]
key-mgmt=wpa-psk
psk=123456789

[ipv4]
method=auto

[ipv6]
method=auto
NMEOF
    sudo chmod 600 "$ROOTFS/etc/NetworkManager/system-connections/ROBO-AI.nmconnection"
fi

# 3. Optionally enable SSH service in rootfs too
if [ -n "$ROOTFS" ]; then
    echo "[3/3] Enabling SSH service in systemd..."
    sudo ln -sf /lib/systemd/system/ssh.service \
        "$ROOTFS/etc/systemd/system/multi-user.target.wants/ssh.service" 2>/dev/null || true
    sudo ln -sf /lib/systemd/system/sshd.service \
        "$ROOTFS/etc/systemd/system/multi-user.target.wants/sshd.service" 2>/dev/null || true
    echo "       SSH systemd service linked."
else
    echo "[3/3] Rootfs not found, skipping systemd link (boot/ssh file should suffice)."
fi

echo ""
echo "============================================="
echo "  DONE! Now:"
echo "  1. Safely eject the SD card"
echo "  2. Put it back in the Pi"
echo "  3. Power on the Pi"
echo "  4. Wait ~30 seconds, then SSH in:"
echo ""
echo "     ssh pi@raspberrypi.local"
echo "     (default password: raspberry)"
echo "============================================="
