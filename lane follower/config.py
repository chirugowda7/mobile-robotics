"""
Configuration settings for Camera True Line Follower with Cytron Motor Driver.
"""
import os
import json

# ==============================================================================
# Hardware GPIO Pin Configuration (Broadcom BCM Pin Numbering)
# Cytron Motor Driver (PWM + DIR mode)
# Common models: MDD10A, MD10C, MDD3A, Maker Drive
# ==============================================================================
# Left Motor Pins
PIN_PWM_LEFT = 18    # BCM 18 (Physical Pin 12, Hardware PWM0)
PIN_DIR_LEFT = 23    # BCM 23 (Physical Pin 16, Digital Output)

# Right Motor Pins
PIN_PWM_RIGHT = 19   # BCM 19 (Physical Pin 35, Hardware PWM1)
PIN_DIR_RIGHT = 24   # BCM 24 (Physical Pin 18, Digital Output)

# PWM Frequency in Hertz (Cytron supports up to 20kHz, 1000Hz is quiet & responsive)
PWM_FREQUENCY = 1000

# Direction Inversions (set to True if motor spins backwards for forward motion)
INVERT_LEFT_MOTOR = False
INVERT_RIGHT_MOTOR = False

# ==============================================================================
# Camera Configuration
# ==============================================================================
CAMERA_INDEX = 0          # Device index (0 for /dev/video0, or depth camera RGB stream)
CAMERA_WIDTH = 640        # Resolution width
CAMERA_HEIGHT = 480       # Resolution height
CAMERA_FPS = 30           # Target FPS

# Region of Interest (ROI) - Bottom percentage of frame to inspect
# 0.50 means the bottom 50% of the image (rows from height*0.5 to height)
ROI_START_RATIO = 0.50
ROI_END_RATIO = 0.95

# Minimum contour area in pixels to be recognized as a valid line (rejects noise)
MIN_CONTOUR_AREA = 500

# ==============================================================================
# Speed and Motion Parameters (Duty Cycle in %)
# ==============================================================================
BASE_SPEED = 35.0         # Default cruising forward speed (0 - 100)
MAX_SPEED = 65.0          # Max motor duty cycle allowed
MIN_SPEED = 0.0           # Minimum speed allowed
LOST_LINE_TIMEOUT = 1.0   # Seconds before full stop if line disappears

# Turning & Pivot Parameters (One motor forward, one in reverse for accurate spin-turn)
PIVOT_TURN_ENABLED = True # Active counter-rotation during turns
TURN_DEADZONE = 0.08      # Straight tolerance (no reverse when centered)
REVERSE_FACTOR = 0.85     # Strength of the reverse inside wheel (0.0 to 1.0)

# ==============================================================================
# PID Steering Controller Gains
# ==============================================================================
# Error is normalized between -1.0 (far left) and +1.0 (far right)
PID_KP = 35.0             # Proportional gain (immediate response to error)
PID_KI = 0.05             # Integral gain (corrects steady-state offset)
PID_KD = 12.0             # Derivative gain (dampens oscillation/overshoot)
PID_MAX_INTEGRAL = 20.0   # Anti-windup clamp for integral term

# ==============================================================================
# HSV Color Ranges for Line Detection
# Format: [H_min, S_min, V_min], [H_max, S_max, V_max]
# OpenCV Hue range is 0 - 179, Saturation is 0 - 255, Value is 0 - 255
# ==============================================================================
DEFAULT_COLOR_RANGES = {
    "yellow": {
        "lower": [20, 80, 100],
        "upper": [35, 255, 255],
        "display_color": (0, 255, 255)  # BGR for UI
    },
    "green": {
        "lower": [35, 50, 50],
        "upper": [85, 255, 255],
        "display_color": (0, 255, 0)
    },
    "blue": {
        "lower": [100, 100, 50],
        "upper": [135, 255, 255],
        "display_color": (255, 120, 0)
    },
    "red": {
        # Red wraps around the 0/180 Hue boundary
        "lower": [0, 120, 70],
        "upper": [10, 255, 255],
        "lower2": [170, 120, 70],
        "upper2": [180, 255, 255],
        "display_color": (0, 0, 255)
    },
    "black": {
        "lower": [0, 0, 0],
        "upper": [180, 255, 60],
        "display_color": (50, 50, 50)
    }
}

CONFIG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "color_config.json")


def load_color_ranges():
    """Loads custom calibrated color ranges from JSON if present, else defaults."""
    if os.path.exists(CONFIG_FILE_PATH):
        try:
            with open(CONFIG_FILE_PATH, "r") as f:
                saved = json.load(f)
                ranges = DEFAULT_COLOR_RANGES.copy()
                for key, val in saved.items():
                    if key in ranges:
                        ranges[key]["lower"] = val.get("lower", ranges[key]["lower"])
                        ranges[key]["upper"] = val.get("upper", ranges[key]["upper"])
                        if "lower2" in val:
                            ranges[key]["lower2"] = val["lower2"]
                        if "upper2" in val:
                            ranges[key]["upper2"] = val["upper2"]
                return ranges
        except Exception as e:
            print(f"[CONFIG] Warning loading {CONFIG_FILE_PATH}: {e}, using defaults.")
    return DEFAULT_COLOR_RANGES.copy()


def save_color_ranges(color_ranges):
    """Saves customized color ranges to JSON file."""
    data_to_save = {}
    for col, info in color_ranges.items():
        entry = {
            "lower": info["lower"],
            "upper": info["upper"]
        }
        if "lower2" in info:
            entry["lower2"] = info["lower2"]
            entry["upper2"] = info["upper2"]
        data_to_save[col] = entry

    with open(CONFIG_FILE_PATH, "w") as f:
        json.dump(data_to_save, f, indent=4)
    print(f"[CONFIG] Color configuration saved to {CONFIG_FILE_PATH}")


def compute_wheel_speeds(base_speed: float, steering: float, error: float):
    """
    Computes left and right motor speeds with active pivot turning.
    When turning:
      - Outer motor drives FORWARD (+speed)
      - Inner motor drives in REVERSE (-speed) for sharp, accurate pivot turns.
      - When centered (|error| <= TURN_DEADZONE), both motors drive forward smoothly.
    """
    if not PIVOT_TURN_ENABLED or abs(error) <= TURN_DEADZONE:
        # Near center: standard smooth forward driving
        left = base_speed + steering
        right = base_speed - steering
        # Keep wheels forward when centered
        left = max(0.0, left)
        right = max(0.0, right)
    elif error > 0:
        # Line is to the RIGHT -> Turn RIGHT
        # Left motor: FORWARD | Right motor: REVERSE
        turn_intensity = min(1.0, (error - TURN_DEADZONE) / (0.70 - TURN_DEADZONE))
        left = base_speed + abs(steering)
        right = - (base_speed * REVERSE_FACTOR * turn_intensity)
    else:
        # Line is to the LEFT -> Turn LEFT
        # Right motor: FORWARD | Left motor: REVERSE
        turn_intensity = min(1.0, (abs(error) - TURN_DEADZONE) / (0.70 - TURN_DEADZONE))
        right = base_speed + abs(steering)
        left = - (base_speed * REVERSE_FACTOR * turn_intensity)

    # Clamp both speeds to maximum allowed speed
    left = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    return left, right

