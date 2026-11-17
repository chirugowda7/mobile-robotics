# Autonomous Camera True Line Follower (Cytron Motor Driver + Raspberry Pi)

A computer vision-based **True Line Follower** using an RGB / Depth Camera, Raspberry Pi, and a **Cytron Motor Driver** (PWM + DIR mode) controlling two DC motors.

Features **dynamic multi-color selection** allowing you to follow **Green**, **Blue**, or **Yellow** lines on the fly, with real-time PID steering and interactive calibration tools.

---

## 🏎️ System Architecture & Working Principle

```
+-------------------------------------------------------------+
|                     Raspberry Pi 4 / 5                      |
|                                                             |
|   +-----------------------+     +-----------------------+   |
|   |  Depth Camera (RGB)   |     |    Color Selector     |   |
|   |  /dev/video0 (640x480)|     | [Yellow, Green, Blue] |   |
|   +-----------+-----------+     +-----------+-----------+   |
|               |                             |               |
|               v                             v               |
|      +-----------------------------------------------+      |
|      | Line Detector: HSV Masking + ROI + Centroid   |      |
|      +-----------------------+-----------------------+      |
|                              | Error: (Cx - Center)         |
|                              v                              |
|      +-----------------------------------------------+      |
|      | PID Steering Controller (Kp, Ki, Kd)          |      |
|      +-----------------------+-----------------------+      |
|                              |                              |
|                              v                              |
|      +-----------------------------------------------+      |
|      | Differential Drive: Left & Right PWM Duty %   |      |
|      +-----------------------+-----------------------+      |
+------------------------------|------------------------------+
                               | GPIO (PWM + DIR)
                               v
               +-------------------------------+
               |      Cytron Motor Driver      |
               | (e.g. MDD10A, MD10C, MDD3A)   |
               +---------------+---------------+
                               |
                   +-----------+-----------+
                   |                       |
                   v                       v
            [Left DC Motor]        [Right DC Motor]
```

---

## 🔌 Hardware Wiring Diagram

### 1. Raspberry Pi to Cytron Motor Driver

| Cytron Pin | Raspberry Pi Pin | BCM GPIO Number | Function |
| :--- | :--- | :--- | :--- |
| **PWM 1 (Left)** | Physical Pin 12 | **BCM 18** (Hardware PWM0) | Left Motor Speed (0 - 100%) |
| **DIR 1 (Left)** | Physical Pin 16 | **BCM 23** | Left Motor Direction (HIGH / LOW) |
| **PWM 2 (Right)**| Physical Pin 35 | **BCM 19** (Hardware PWM1) | Right Motor Speed (0 - 100%) |
| **DIR 2 (Right)**| Physical Pin 18 | **BCM 24** | Right Motor Direction (HIGH / LOW) |
| **GND** | Physical Pin 6, 9, or 14 | **GND** | **Common Ground (Crucial!)** |

> ⚠️ **IMPORTANT**: Ensure the Raspberry Pi **GND** is connected to the Cytron **GND** pin to establish a shared signal reference.

### 2. Cytron Power & Motors

- **POWER + (V+) / POWER - (GND)**: Connect to your robot battery pack (e.g., 2S/3S LiPo or 7.4V - 12V pack).
- **M1A / M1B**: Connect to Left DC Motor.
- **M2A / M2B**: Connect to Right DC Motor.

---

## 📦 Installation & Setup on Raspberry Pi

1. **Clone or copy this repository to your Raspberry Pi**:
   ```bash
   cd ~/lane\ follower
   ```

2. **Install system dependencies**:
   ```bash
   sudo apt-get update
   sudo apt-get install -y python3-opencv python3-numpy python3-rpi.gpio
   ```

   *(On Raspberry Pi OS Bookworm / Pi 5, install `python3-gpiozero` or `python3-lgpio` if preferred).*

---

## 🎮 How to Use

### 1. Pre-Flight Test: Test Motor Wiring
Before putting the robot on the track, verify that left and right motors spin in the correct directions:
```bash
python3 test_motors.py
```
- Press `1` to test Left Motor, `3` to test Right Motor, and `5` for Both Forward.
- If a motor spins backwards when told forward, toggle `INVERT_LEFT_MOTOR = True` or `INVERT_RIGHT_MOTOR = True` in [config.py](file:///home/robotics/lane%20follower/config.py).

---

### 2. Run Autonomous Line Follower
Run the main tracking application with your initial target color:

```bash
# Follow Yellow Line (Default)
python3 line_follower.py --color yellow

# Follow Green Line
python3 line_follower.py --color green

# Follow Blue Line
python3 line_follower.py --color blue
```

#### Real-Time Keyboard Controls (during operation):
| Key | Action |
|:---:|:---|
| **`Y`** | Instantly switch target to **Yellow** line |
| **`G`** | Instantly switch target to **Green** line |
| **`B`** | Instantly switch target to **Blue** line |
| **`C`** | Cycle through all configured colors |
| **`SPACE`** | **Emergency Pause / Resume** motor movement |
| **`+` / `-`** | Increase / Decrease base cruising speed |
| **`Q` / `ESC`** | Safely stop motors, release camera, and exit |

#### Headless Mode (For SSH without Monitor):
```bash
python3 line_follower.py --color green --headless
```

---

### 3. Ambient Light Calibration Tool
If the room lighting changes or the camera is struggling to pick up the line:
```bash
python3 calibrate_color.py --color yellow
```
- Interactive sliders will appear for `H Min`, `H Max`, `S Min`, `S Max`, `V Min`, `V Max`.
- Adjust sliders until the **CLEANED MASK** window isolates only the line and ignores background noise.
- Press **`S`** to permanently save the calibrated values to `color_config.json`.
- Press **`1`**, **`2`**, or **`3`** to switch between Yellow, Green, and Blue calibrations without restarting!

---

## ⚙️ Tuning Parameters ([config.py](file:///home/robotics/lane%20follower/config.py))

- **`BASE_SPEED`** (Default `35%`): Forward speed. Lower it for tight tracks, increase for fast straights.
- **`ROI_START_RATIO`** (Default `0.50`): The bottom 50% of the image is used to focus on the ground line and filter out background lights.
- **`PID_KP`** (Default `35.0`): Proportional gain. Increase if the robot reacts too sluggishly on turns.
- **`PID_KD`** (Default `12.0`): Derivative gain. Increase to dampen fish-tailing / wobbling oscillations.
- **`LOST_LINE_TIMEOUT`** (Default `1.0s`): If the line disappears, the robot attempts recovery for 1 second, then stops automatically.
