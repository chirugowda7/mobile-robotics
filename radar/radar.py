"""
=============================================================================
  RADAR DETECTION SYSTEM - MATPLOTLIB GUI (20 CM CLOSE-RANGE)
=============================================================================
  Compatible with Arduino Uno + HC-SR04 + MG996R / SG90
  Auto-detects Arduino COM port (filters out non-responsive Bluetooth ports)
  Matches Arduino Sketch:
    - Baud Rate: 9600
    - Range: 20 cm
    - Data Format: angle,distance_cm,x_cm,y_cm
    - Summary Format: NEAREST,nearestAngle,nearestDist
=============================================================================
"""

import time
import math
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

# ---------- Settings ----------
COM_PORT   = "AUTO"      # "AUTO" will automatically detect your Arduino, or set e.g. "COM15"
BAUD_RATE  = 9600
MAX_RANGE  = 20          # 20 cm close-range detection
MIN_ANGLE  = 15
MAX_ANGLE  = 165
STEP_ANGLE = 5

# ---------- Smart COM Port Auto-Detection ----------
def detect_arduino_port():
    """Finds the real Arduino USB serial port and filters out dead Bluetooth links."""
    all_ports = serial.tools.list_ports.comports()
    
    # 1. Look for known Arduino / USB serial hardware
    for p in all_ports:
        desc = p.description.lower()
        hwid = p.hwid.lower()
        if any(keyword in desc or keyword in hwid for keyword in ["arduino", "ch340", "cp210", "ftdi", "usb serial"]):
            return p.device, p.description

    # 2. Look for any active serial port that is NOT Bluetooth
    for p in all_ports:
        if "bthenum" not in p.hwid.lower() and "bluetooth" not in p.description.lower():
            return p.device, p.description

    return None, None

# Attempt connection
ser = None
is_demo_mode = False
target_port = COM_PORT

if target_port == "AUTO":
    detected_port, port_desc = detect_arduino_port()
    if detected_port:
        target_port = detected_port
        print(f"[Serial] Auto-detected Arduino on: {target_port} ({port_desc})")
    else:
        print("[Serial] [NOTICE] No Arduino USB port detected. Starting in DEMO SCAN MODE.")
        is_demo_mode = True

if not is_demo_mode:
    try:
        ser = serial.Serial(target_port, BAUD_RATE, timeout=0.1)
        print(f"[Serial] Successfully connected to {target_port} @ {BAUD_RATE} baud.")
    except Exception as e:
        print(f"[Serial] [WARNING] Could not connect to {target_port}: {e}")
        print("[Serial] Falling back to DEMO SCAN MODE.")
        is_demo_mode = True

# ---------- Radar State ----------
angles = []        # sweep angles (radians)
distances = []     # smoothed distances (cm)
current_angle = MIN_ANGLE
current_dist = 0.0
nearest_text = "Nearest: --"
trail_points = []  # (angle_rad, dist) for real obstacle hits

# Simulation / Demo state for 20cm range
sim_angle = MIN_ANGLE
sim_dir = STEP_ANGLE
sim_last_time = time.time()
sim_obstacles = [
    {"angle": 45,  "span": 12, "dist": 8.5},
    {"angle": 90,  "span": 16, "dist": 14.2},
    {"angle": 135, "span": 12, "dist": 11.0},
]

# ---------- Plot Setup (20 cm Scale) ----------
fig = plt.figure(figsize=(9, 7), facecolor="#080e14")
ax = fig.add_subplot(111, polar=True, facecolor="#080e14")

ax.set_thetamin(MIN_ANGLE)
ax.set_thetamax(MAX_ANGLE)
ax.set_theta_zero_location("W")   # 0 deg on the left, 180 on the right
ax.set_theta_direction(-1)        # sweep clockwise
ax.set_ylim(0, MAX_RANGE)

# Range rings at 5cm, 10cm, 15cm, 20cm
ax.set_rticks([5, 10, 15, 20])
ax.set_yticklabels(["5", "10", "15", "20 cm"], color="#00ff66", fontsize=9, fontweight="bold")
ax.set_xticks([math.radians(a) for a in range(MIN_ANGLE, MAX_ANGLE + 1, 30)])
ax.tick_params(colors="#00ff66")
ax.grid(color="#00ff66", alpha=0.35, linestyle="--")
ax.spines["polar"].set_color("#00ff66")

# Visual Elements
sweep_line, = ax.plot([], [], color="#00ff66", linewidth=2.5, label="Sweep Beam")
hits, = ax.plot([], [], "o", color="#ff3333", markersize=8, label="Detected Objects")
trail, = ax.plot([], [], "o", color="#ffaa00", alpha=0.45, markersize=5)
status = ax.text(0.5, -0.12, "", transform=ax.transAxes,
                 ha="center", color="#00ff88", fontsize=11, fontweight="bold")

mode_banner = "[LIVE HARDWARE]" if not is_demo_mode else "[DEMO SCAN - CONNECT ARDUINO TO USB]"
title_obj = plt.title(f"Ultrasonic Radar (20cm Range) {mode_banner}", color="#00ff88", pad=20, fontsize=13, fontweight="bold")

# ---------- Data Processing ----------
def read_serial_data():
    """Reads and parses incoming serial lines from the Arduino."""
    global current_angle, current_dist, nearest_text, ser, is_demo_mode

    # If in demo mode, periodically check if an Arduino was newly plugged in
    if is_demo_mode:
        auto_p, _ = detect_arduino_port()
        if auto_p:
            try:
                ser = serial.Serial(auto_p, BAUD_RATE, timeout=0.1)
                is_demo_mode = False
                print(f"[Serial] Arduino detected and connected on {auto_p}!")
                title_obj.set_text(f"Ultrasonic Radar (20cm Range) [LIVE HARDWARE: {auto_p}]")
            except Exception:
                pass

    if is_demo_mode:
        sim_step()
        return

    if not ser or not ser.is_open:
        return

    try:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line or line.startswith("angle"):
                continue

            if line.startswith("NEAREST"):
                parts = line.split(",")
                if len(parts) >= 3:
                    nearest_text = f"Nearest: {parts[2]} cm @ {parts[1]}°"
                continue

            parts = line.split(",")
            if len(parts) >= 2:
                try:
                    a = int(float(parts[0]))
                    d = float(parts[1])
                except ValueError:
                    continue

                current_angle = a
                current_dist = d

                # Record detection point
                rad_a = math.radians(a)
                angles.append(rad_a)
                distances.append(d)

                # Real hit threshold (within 20cm, excluding 'no echo' at >= 19.5cm)
                if d < (MAX_RANGE - 0.5):
                    trail_points.append((rad_a, d))
                    if len(trail_points) > 100:
                        trail_points.pop(0)

                # Keep only latest sweep degrees
                if len(angles) > 181:
                    angles.pop(0)
                    distances.pop(0)

    except serial.SerialException:
        print("[Serial] Connection lost. Reverting to Demo Mode...")
        is_demo_mode = True
        title_obj.set_text("Ultrasonic Radar (20cm Range) [DEMO SCAN - RECONNECTING]")


def sim_step():
    """Simulates realistic radar sweeps when no Arduino is connected."""
    global sim_angle, sim_dir, sim_last_time, current_angle, current_dist, nearest_text
    now = time.time()
    if now - sim_last_time >= 0.08:  # 80ms step delay matching Arduino
        sim_last_time = now
        sim_angle += sim_dir

        if sim_angle >= MAX_ANGLE:
            sim_angle = MAX_ANGLE
            sim_dir = -STEP_ANGLE
            nearest_text = "Nearest: 8.5 cm @ 45°"
        elif sim_angle <= MIN_ANGLE:
            sim_angle = MIN_ANGLE
            sim_dir = STEP_ANGLE

        current_angle = sim_angle

        # Simulated distance within 20cm
        dist = MAX_RANGE
        for obs in sim_obstacles:
            if abs(sim_angle - obs["angle"]) <= (obs["span"] / 2):
                noise = math.sin(time.time() * 5 + sim_angle) * 0.4
                dist = obs["dist"] + noise
                break

        current_dist = dist
        rad_a = math.radians(sim_angle)
        angles.append(rad_a)
        distances.append(dist)

        if dist < (MAX_RANGE - 0.5):
            trail_points.append((rad_a, dist))
            if len(trail_points) > 100:
                trail_points.pop(0)

        if len(angles) > 181:
            angles.pop(0)
            distances.pop(0)


def update(frame):
    read_serial_data()

    # Sweep line follows the current angle
    rad = math.radians(current_angle)
    sweep_line.set_data([rad, rad], [0, MAX_RANGE])

    # Show only real hits within 20 cm
    hit_angles = [a for a, d in zip(angles, distances) if d < (MAX_RANGE - 0.5)]
    hit_dists  = [d for d in distances if d < (MAX_RANGE - 0.5)]
    hits.set_data(hit_angles, hit_dists)

    # Fading trail of detected hits
    if trail_points:
        ta, td = zip(*trail_points)
        trail.set_data(ta, td)
    else:
        trail.set_data([], [])

    # Format telemetry display
    dist_display = f"{current_dist:.1f} cm" if current_dist < (MAX_RANGE - 0.5) else "CLEAR (>20 cm)"
    status.set_text(f"{nearest_text}   |   Current: {current_angle:3d}° @ {dist_display}")

    return sweep_line, hits, trail, status


# ---------- Interactive Refresh Button & Keyboard Shortcuts ----------
# Adjust margins to leave dedicated space for the refresh button
plt.subplots_adjust(bottom=0.16, top=0.88)

# Add Refresh Button: [left, bottom, width, height]
ax_btn = fig.add_axes([0.38, 0.025, 0.24, 0.05])
btn_refresh = Button(
    ax_btn,
    '↻ Refresh Data',
    color='#102b1c',
    hovercolor='#1a4a2f'
)
btn_refresh.label.set_color('#00ff88')
btn_refresh.label.set_fontsize(11)
btn_refresh.label.set_fontweight('bold')

def refresh_sensed_data(event=None):
    """Refreshes and clears all sensed radar data and buffers."""
    global nearest_text, current_dist
    angles.clear()
    distances.clear()
    trail_points.clear()
    nearest_text = "Nearest: --"
    current_dist = float(MAX_RANGE)

    # Discard old queued bytes in the serial buffer so fresh sweep data is processed
    if ser and ser.is_open:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

    # Clear visual plot elements immediately
    hits.set_data([], [])
    trail.set_data([], [])
    status.set_text("Radar Data Refreshed! Rescanning...")
    fig.canvas.draw_idle()
    print("[Radar] Sensed data, hits, and buffer refreshed successfully.")

btn_refresh.on_clicked(refresh_sensed_data)

# Keyboard shortcut: Press 'R' on keyboard to also trigger refresh
def on_key(event):
    if event.key in ('r', 'R'):
        refresh_sensed_data()

fig.canvas.mpl_connect('key_press_event', on_key)

ani = FuncAnimation(fig, update, interval=40, cache_frame_data=False)
plt.show()

if ser and ser.is_open:
    ser.close()
