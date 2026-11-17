"""
6-Axis Arm Controller with Vision Sorting (J6 Home: 45° | 40°–100° Range)
-------------------------------------------------------------------------
Features:
  - Exact layout & design matching axis.py (Left Column)
  - Integrated Vision & Auto-Sorting Station (Right Column)
  - Zero-Lag Background Threaded Camera Engine (cap.read never blocks UI)
  - Canvas image-item reuse (zero memory leak, silky 60 FPS)
  - Live PID smooth slider control at 115200 baud
  - Motion recorder with auto-save & file manager
  - 1-Click Color Assignment (Red, Green, Blue, Yellow)
  - Autonomous Color Sorting mode
  - Strict J6 range lock: 40° to 100°, Safe Home: [90, 80, 108, 38, 96, 45]
"""

import json
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# Computer Vision & Pillow
CV_AVAILABLE = True
try:
    import cv2
    import numpy as np
    from PIL import Image, ImageTk
except ImportError:
    CV_AVAILABLE = False

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------- Configuration
NUM_JOINTS = 6

# Individual Joint Limits: (min_deg, max_deg)
JOINT_LIMITS = [
    (0.0, 180.0),   # J1: Base Turntable
    (0.0, 180.0),   # J2: Shoulder
    (0.0, 180.0),   # J3: Elbow
    (0.0, 180.0),   # J4: Wrist Pitch
    (0.0, 180.0),   # J5: Wrist Roll
    (40.0, 100.0),  # J6: Gripper / Claw (Locked to 40°–100°)
]

# Safe Home Position per Joint: [J1, J2, J3, J4, J5, J6]
HOME_POSITION = [90.0, 80.0, 108.0, 38.0, 96.0, 45.0]

DEFAULT_BAUD = 115200
LIVE_DEBOUNCE_MS = 25  # ~40 Hz refresh for smooth live tracking

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
os.makedirs(RECORDINGS_DIR, exist_ok=True)
CONFIG_FILE = os.path.join(SCRIPT_DIR, "vision_mapping.json")

COLORS = ["RED", "GREEN", "BLUE", "YELLOW"]
COLOR_HEX = {
    "RED": "#d9534f",
    "GREEN": "#28a745",
    "BLUE": "#0275d8",
    "YELLOW": "#f0ad4e",
    "NONE": "#6c757d"
}


def build_command(angles):
    """Format joint angles to the serial command protocol."""
    return "MOVE " + ",".join(f"{a:.2f}" for a in angles) + "\n"


# ---------------------------------------------------------------- Serial Link
class SerialLink:
    """Owns the serial port and background reader thread."""
    def __init__(self, rx_queue: queue.Queue):
        self.rx_queue = rx_queue
        self.ser = None
        self._thread = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)  # Microcontroller auto-reset wait
        self.ser.reset_input_buffer()
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def send(self, text: str):
        if not self.connected:
            raise ConnectionError("Not connected to a serial port.")
        with self._write_lock:
            self.ser.write(text.encode("ascii"))

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                waiting = self.ser.in_waiting
                data = self.ser.read(waiting or 1)
            except Exception as exc:
                self.rx_queue.put(("error", f"Serial error: {exc}"))
                return
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.rx_queue.put(("rx", line.decode(errors="replace").strip()))


# ---------------------------------------------------------------- Background Threaded Camera
class BackgroundCamera:
    """Non-blocking background video capture engine to eliminate all GUI lag."""
    def __init__(self):
        self.cap = None
        self.latest_frame = None
        self.active_idx = 0
        self.is_running = False
        self._thread = None
        self._lock = threading.Lock()

    def start(self, camera_idx=2):
        self.stop()
        self.active_idx = camera_idx

        # 1. Try DirectShow with MJPG
        backend = cv2.CAP_DSHOW if os.name == 'nt' else 0
        cap = cv2.VideoCapture(camera_idx, backend)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 2. Fallback to default if DirectShow fails
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(camera_idx)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            return False

        self.cap = cap
        self.is_running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        return True

    def _worker(self):
        while self.is_running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None and frame.size > 0:
                with self._lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.02)

    def get_latest_frame(self):
        with self._lock:
            if self.latest_frame is not None:
                return True, self.latest_frame.copy()
            return False, None

    def stop(self):
        self.is_running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.4)
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        with self._lock:
            self.latest_frame = None


# ---------------------------------------------------------------- Color Detector
class ColorDetector:
    """HSV color detection in center Pick Zone."""
    def __init__(self):
        self.ranges = {
            "RED": [
                ((0, 115, 80), (10, 255, 255)),
                ((165, 115, 80), (180, 255, 255))
            ],
            "GREEN": [
                ((35, 75, 70), (85, 255, 255))
            ],
            "BLUE": [
                ((95, 80, 70), (135, 255, 255))
            ],
            "YELLOW": [
                ((18, 90, 90), (34, 255, 255))
            ]
        }

    def detect(self, bgr_roi):
        if bgr_roi is None or bgr_roi.size == 0:
            return "NONE", 0.0

        hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
        total_pixels = bgr_roi.shape[0] * bgr_roi.shape[1]
        best_color = "NONE"
        max_ratio = 0.0

        for color_name, bounds in self.ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in bounds:
                m = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
                mask = cv2.bitwise_or(mask, m)

            matched = cv2.countNonZero(mask)
            ratio = matched / float(total_pixels)
            if ratio > 0.14 and ratio > max_ratio:
                max_ratio = ratio
                best_color = color_name

        return best_color, max_ratio


# ---------------------------------------------------------------- Main Application GUI
class VisionArmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("6-Axis Arm Controller (J6 Home: 45° | 40°–100° Range)")
        self.resizable(False, False)

        self.rx_queue = queue.Queue()
        self.link = SerialLink(self.rx_queue)

        # Background Camera & Vision
        self.cam = BackgroundCamera() if CV_AVAILABLE else None
        self.detector = ColorDetector() if CV_AVAILABLE else None
        self.canvas_img_id = None
        self.active_cam_idx = 2  # Prioritize external USB webcam
        self.current_detected_color = "NONE"
        self.color_confidence = 0.0

        # Sliders & Live Control
        self.angle_vars: list[tk.DoubleVar] = []
        self._live_job = None
        self.speed_var = tk.IntVar(value=65)

        # Recorder & Playback
        self.recording: list[tuple[float, list[float]]] = []
        self._is_recording = False
        self._record_start_time = 0.0
        self._is_playing = False
        self._playback_thread = None
        self._stop_playback_event = threading.Event()
        self.loop_var = tk.BooleanVar(value=False)
        self.selected_file_var = tk.StringVar()

        # Auto-Sorting State
        self.auto_sort_active = True  # Automatically active by default!
        self.stable_color_counter = 0
        self.last_seen_color = "NONE"
        self.last_sorted_time = 0.0
        self.color_mappings = {c: tk.StringVar(value="") for c in COLORS}

        # UI String Vars
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.rec_status_var = tk.StringVar(value="Recorder: Idle (0 points)")
        self.auto_status_var = tk.StringVar(value="Auto-Sort: ACTIVE (Place product in Pick Zone)")

        # Build Side-by-Side Dashboard
        self._build_ui()
        self.load_saved_mappings()
        self.refresh_ports()
        self.refresh_saved_files()
        self.refresh_camera_list()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(50, self._poll_rx)

        # Start Camera & Vision loop
        if CV_AVAILABLE:
            self.after(200, self.start_camera)
            self.after(250, self._update_camera_feed)

    # ---------- UI Construction
    def _build_ui(self):
        pad = {"padx": 5, "pady": 3}

        # Root frame with 2 columns:
        # Left Column: The EXACT Arm Controller from axis.py
        # Right Column: The Vision & Auto-Sorting Panel
        main_box = ttk.Frame(self)
        main_box.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        left_col = ttk.Frame(main_box)
        left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        right_col = ttk.Frame(main_box)
        right_col.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        # =========================================================================
        # LEFT COLUMN: EXACT AXIS.PY LAYOUT & DESIGN
        # =========================================================================

        # 1. Connection bar
        conn = ttk.LabelFrame(left_col, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))

        ttk.Label(conn, text="Port:").grid(row=0, column=0, **pad)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=14, state="readonly")
        self.port_combo.grid(row=0, column=1, **pad)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, **pad)

        ttk.Label(conn, text="Baud:").grid(row=0, column=3, **pad)
        ttk.Entry(conn, textvariable=self.baud_var, width=9).grid(row=0, column=4, **pad)

        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, **pad)

        ttk.Label(conn, textvariable=self.status_var).grid(row=1, column=0, columnspan=6, sticky="w", **pad)

        # 2. Joint sliders (Live Control)
        joints = ttk.LabelFrame(left_col, text="Joint Angles (Live Control)")
        joints.grid(row=1, column=0, sticky="ew", padx=8, pady=4)

        for i in range(NUM_JOINTS):
            lo, hi = JOINT_LIMITS[i]
            home_val = HOME_POSITION[i]
            var = tk.DoubleVar(value=home_val)
            var.trace_add("write", self._on_angle_changed)
            self.angle_vars.append(var)

            label_name = f"J{i+1}"
            if i == 5:
                label_name = "J6 (Grip)"

            ttk.Label(joints, text=label_name, width=8).grid(row=i, column=0, **pad)
            scale = tk.Scale(
                joints, from_=lo, to=hi, orient=tk.HORIZONTAL, resolution=1.0,
                showvalue=False, variable=var, length=320
            )
            scale.grid(row=i, column=1, **pad)

            entry = ttk.Entry(joints, textvariable=var, width=7, justify="right")
            entry.grid(row=i, column=2, **pad)
            entry.bind("<Return>", lambda _e: self._trigger_send_now())

            range_text = f"[{int(lo)}°, {int(hi)}°]  (Home: {home_val:g}°)"
            ttk.Label(joints, text=range_text).grid(row=i, column=3, **pad)

        # 3. Motion Recorder & Saved Files Panel
        rec_frame = ttk.LabelFrame(left_col, text="Motion Recorder & Saved Files")
        rec_frame.grid(row=2, column=0, sticky="ew", padx=8, pady=4)

        # Row 0: Controls
        self.rec_btn = tk.Button(
            rec_frame, text="● Record", bg="#c9302c", fg="white",
            font=("TkDefaultFont", 9, "bold"), width=12, command=self.toggle_record
        )
        self.rec_btn.grid(row=0, column=0, **pad)

        self.play_btn = tk.Button(
            rec_frame, text="▶ Play", bg="#28a745", fg="white",
            font=("TkDefaultFont", 9, "bold"), width=12, command=self.toggle_play
        )
        self.play_btn.grid(row=0, column=1, **pad)

        ttk.Checkbutton(rec_frame, text="Loop", variable=self.loop_var).grid(row=0, column=2, **pad)
        ttk.Button(rec_frame, text="Save As...", command=self.save_recording_as).grid(row=0, column=3, **pad)
        ttk.Button(rec_frame, text="Clear", command=self.clear_recording).grid(row=0, column=4, **pad)
        ttk.Button(rec_frame, text="📂 Open Folder", command=self.open_recordings_folder).grid(row=0, column=5, **pad)

        # Row 1: Saved Files Selector
        ttk.Label(rec_frame, text="Saved Files:").grid(row=1, column=0, sticky="e", **pad)
        self.files_combo = ttk.Combobox(
            rec_frame, textvariable=self.selected_file_var, width=28, state="readonly"
        )
        self.files_combo.grid(row=1, column=1, columnspan=2, sticky="ew", **pad)
        self.files_combo.bind("<<ComboboxSelected>>", self.on_file_selected)

        ttk.Button(rec_frame, text="Load File", command=self.load_selected_file).grid(row=1, column=3, **pad)
        ttk.Button(rec_frame, text="Delete File", command=self.delete_selected_file).grid(row=1, column=4, **pad)
        ttk.Button(rec_frame, text="🔄", width=3, command=self.refresh_saved_files).grid(row=1, column=5, **pad)

        # Row 2: Status
        ttk.Label(rec_frame, textvariable=self.rec_status_var, font=("TkDefaultFont", 9, "italic")).grid(
            row=2, column=0, columnspan=6, sticky="w", **pad
        )

        # 4. PID Speed & Commands
        cmd = ttk.LabelFrame(left_col, text="PID Speed & Commands")
        cmd.grid(row=3, column=0, sticky="ew", padx=8, pady=4)

        home_btn = ttk.Button(cmd, text="🏠 Safe Home Position", command=self.go_home)
        home_btn.grid(row=0, column=0, **pad)

        stop_btn = tk.Button(
            cmd, text="EMERGENCY STOP", bg="#d9534f", fg="white",
            font=("TkDefaultFont", 9, "bold"), command=self.emergency_stop
        )
        stop_btn.grid(row=0, column=1, padx=10, pady=4)

        ttk.Label(cmd, text="Speed:").grid(row=0, column=2, **pad)
        speed_scale = tk.Scale(
            cmd, from_=10, to=100, orient=tk.HORIZONTAL, resolution=5,
            showvalue=False, variable=self.speed_var, command=self._on_speed_changed, length=130
        )
        speed_scale.grid(row=0, column=3, **pad)

        self.speed_label = ttk.Label(cmd, text="65% (Smooth)", width=15)
        self.speed_label.grid(row=0, column=4, **pad)

        # 5. Activity Log
        log_frame = ttk.LabelFrame(left_col, text="Log")
        log_frame.grid(row=4, column=0, sticky="nsew", padx=8, pady=(4, 8))
        self.log_text = tk.Text(log_frame, height=7, width=68, state="disabled")
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")

        # =========================================================================
        # RIGHT COLUMN: VISION-GUIDED SORTING STATION
        # =========================================================================

        # 1. USB Webcam Feed (Pick Zone)
        cam_frame = ttk.LabelFrame(right_col, text="USB Webcam (Pick Zone)")
        cam_frame.grid(row=0, column=0, sticky="nsew", padx=8, pady=(8, 4))

        self.video_canvas = tk.Canvas(cam_frame, width=380, height=280, bg="#1a1a1a")
        self.video_canvas.grid(row=0, column=0, columnspan=4, padx=4, pady=4)
        self.canvas_img_id = self.video_canvas.create_image(0, 0, anchor=tk.NW)

        # Cam selector bar
        ttk.Label(cam_frame, text="Cam:").grid(row=1, column=0, padx=2, pady=2)
        self.cam_combo = ttk.Combobox(cam_frame, width=18, state="readonly")
        self.cam_combo.grid(row=1, column=1, padx=2, pady=2)
        self.cam_combo.bind("<<ComboboxSelected>>", self.on_camera_selected)

        ttk.Button(cam_frame, text="🔄", width=3, command=self.refresh_camera_list).grid(row=1, column=2, padx=2, pady=2)
        self.cam_toggle_btn = ttk.Button(cam_frame, text="Start/Stop", width=9, command=self.toggle_camera)
        self.cam_toggle_btn.grid(row=1, column=3, padx=2, pady=2)

        # 2. Product Color Inspection Card
        badge_frame = ttk.LabelFrame(right_col, text="Product Color Inspection")
        badge_frame.grid(row=1, column=0, sticky="ew", padx=8, pady=4)

        self.color_badge = tk.Label(
            badge_frame, text="NO PRODUCT DETECTED", font=("TkDefaultFont", 10, "bold"),
            bg="#555555", fg="white", height=2
        )
        self.color_badge.grid(row=0, column=0, sticky="ew", padx=6, pady=4)
        badge_frame.columnconfigure(0, weight=1)

        # 3. 1-Click Color Assign Buttons
        assign_box = ttk.LabelFrame(right_col, text="1-Click Assign Current Motion to Color:")
        assign_box.grid(row=2, column=0, sticky="ew", padx=8, pady=4)

        tk.Button(
            assign_box, text="→ RED", bg=COLOR_HEX["RED"], fg="white", font=("TkDefaultFont", 8, "bold"),
            command=lambda: self.quick_assign_to_color("RED")
        ).grid(row=0, column=0, padx=2, pady=3)

        tk.Button(
            assign_box, text="→ GREEN", bg=COLOR_HEX["GREEN"], fg="white", font=("TkDefaultFont", 8, "bold"),
            command=lambda: self.quick_assign_to_color("GREEN")
        ).grid(row=0, column=1, padx=2, pady=3)

        tk.Button(
            assign_box, text="→ BLUE", bg=COLOR_HEX["BLUE"], fg="white", font=("TkDefaultFont", 8, "bold"),
            command=lambda: self.quick_assign_to_color("BLUE")
        ).grid(row=0, column=2, padx=2, pady=3)

        tk.Button(
            assign_box, text="→ YELLOW", bg=COLOR_HEX["YELLOW"], fg="white", font=("TkDefaultFont", 8, "bold"),
            command=lambda: self.quick_assign_to_color("YELLOW")
        ).grid(row=0, column=3, padx=2, pady=3)

        # 4. Color-to-Motion Mappings Table
        map_box = ttk.LabelFrame(right_col, text="Color to Trajectory Mappings")
        map_box.grid(row=3, column=0, sticky="ew", padx=8, pady=4)

        self.mapping_combos = {}
        for r, col in enumerate(COLORS):
            lbl = tk.Label(
                map_box, text=f" {col} ", font=("TkDefaultFont", 8, "bold"),
                bg=COLOR_HEX[col], fg="white", width=7
            )
            lbl.grid(row=r, column=0, padx=3, pady=2)

            combo = ttk.Combobox(
                map_box, textvariable=self.color_mappings[col], width=18, state="readonly"
            )
            combo.grid(row=r, column=1, padx=2, pady=2)
            self.mapping_combos[col] = combo

            test_btn = ttk.Button(
                map_box, text="▶", width=3,
                command=lambda c=col: self.test_playback_color(c)
            )
            test_btn.grid(row=r, column=2, padx=2, pady=2)

        ttk.Button(map_box, text="💾 Save Mappings", command=self.save_mappings).grid(
            row=len(COLORS), column=1, sticky="e", padx=2, pady=3
        )

        # 5. Autonomous Color Sorting Controls
        auto_box = ttk.LabelFrame(right_col, text="Sorting Execution")
        auto_box.grid(row=4, column=0, sticky="ew", padx=8, pady=(4, 8))

        self.sort_now_btn = tk.Button(
            auto_box, text="🎯 Sort Current Product Now", font=("TkDefaultFont", 9, "bold"),
            bg="#0275d8", fg="white", command=self.trigger_manual_sort
        )
        self.sort_now_btn.grid(row=0, column=0, columnspan=2, sticky="ew", padx=4, pady=2)

        self.auto_btn = tk.Button(
            auto_box, text="■ Pause Auto-Sorting", font=("TkDefaultFont", 9, "bold"),
            bg="#c9302c", fg="white", command=self.toggle_auto_sort
        )
        self.auto_btn.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=2)

        ttk.Label(auto_box, textvariable=self.auto_status_var, font=("TkDefaultFont", 8, "italic")).grid(
            row=2, column=0, columnspan=2, sticky="w", padx=4, pady=2
        )

    # ---------- Helpers
    def log(self, msg: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {msg}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self.log(f"Found ports: {', '.join(ports) if ports else 'none'}")

    def refresh_saved_files(self, select_filename=None):
        try:
            files = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith(".json")]
            files.sort(key=lambda f: os.path.getmtime(os.path.join(RECORDINGS_DIR, f)), reverse=True)
            self.files_combo["values"] = files
            if files:
                if select_filename and select_filename in files:
                    self.files_combo.set(select_filename)
                elif not self.selected_file_var.get() or self.selected_file_var.get() not in files:
                    self.files_combo.set(files[0])
            else:
                self.files_combo.set("")

            # Update color mapping dropdowns
            options = ["(None)"] + files
            for combo in self.mapping_combos.values():
                combo["values"] = options
        except Exception as e:
            self.log(f"Error refreshing recordings: {e}")

    def read_angles(self) -> list[float]:
        angles = []
        for i, var in enumerate(self.angle_vars):
            try:
                v = float(var.get())
            except (tk.TclError, ValueError):
                raise ValueError(f"J{i+1} is not a valid number.")
            lo, hi = JOINT_LIMITS[i]
            if not lo <= v <= hi:
                raise ValueError(f"J{i+1} = {v:g}° is outside allowed range [{lo:g}°, {hi:g}°].")
            angles.append(v)
        return angles

    def _on_speed_changed(self, val):
        pct = int(float(val))
        label_text = f"{pct}% (Gentle)" if pct <= 45 else (f"{pct}% (Smooth)" if pct <= 75 else f"{pct}% (Fast)")
        self.speed_label.configure(text=label_text)
        if self.link.connected:
            try:
                self.link.send(f"SPEED {pct}\n")
            except Exception:
                pass

    # ---------- Connection
    def toggle_connection(self):
        if self.link.connected:
            self.stop_playback()
            if self._is_recording:
                self.toggle_record()
            self.link.disconnect()
            self.connect_btn.configure(text="Connect")
            self.status_var.set("Disconnected")
            self.log("Disconnected.")
            return

        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a COM port first.")
            return
        try:
            baud = int(self.baud_var.get())
            self.status_var.set(f"Connecting to {port}...")
            self.update_idletasks()
            self.link.connect(port, baud)
        except (ValueError, serial.SerialException) as exc:
            self.status_var.set("Disconnected")
            messagebox.showerror("Connection failed", str(exc))
            return

        self.connect_btn.configure(text="Disconnect")
        self.status_var.set(f"Connected: {port} @ {baud} (Safe Home Active)")
        self.log(f"Connected to {port} at {baud} baud. J6 Home is 45°.")

        try:
            self.link.send(f"SPEED {self.speed_var.get()}\n")
        except Exception:
            pass
        self.go_home()

    # ---------- Live Slider Movement
    def _on_angle_changed(self, *_):
        if self._is_playing or not self.link.connected:
            return
        if self._live_job is not None:
            self.after_cancel(self._live_job)
        self._live_job = self.after(LIVE_DEBOUNCE_MS, self._live_send)

    def _trigger_send_now(self):
        if self._live_job is not None:
            self.after_cancel(self._live_job)
            self._live_job = None
        self._live_send()

    def _live_send(self):
        self._live_job = None
        if not self.link.connected:
            return
        try:
            angles = self.read_angles()
        except (ValueError, tk.TclError):
            return
        try:
            self.link.send(build_command(angles))
            if self._is_recording:
                t = time.time() - self._record_start_time
                self.recording.append((t, list(angles)))
                self.rec_status_var.set(f"Recording: {len(self.recording)} pts ({t:.1f}s)")
        except (ConnectionError, serial.SerialException, OSError) as exc:
            self.log(f"Live send failed: {exc}")

    def go_home(self):
        for i, h in enumerate(HOME_POSITION):
            self.angle_vars[i].set(h)
        if self.link.connected:
            try:
                self.link.send(build_command(HOME_POSITION))
                self.log(f"Moved to Safe Home: {HOME_POSITION}")
            except Exception as e:
                self.log(f"Home failed: {e}")

    def emergency_stop(self):
        self.stop_playback()
        if self._is_recording:
            self.toggle_record()
        self.auto_sort_active = False
        self.auto_status_var.set("Auto-Sort: Stopped (E-Stop)")
        self.auto_btn.configure(text="🔄 Start Auto-Sorting Mode", bg="#28a745")
        if self.link.connected:
            try:
                self.link.send("STOP\n")
                self.log("EMERGENCY STOP sent.")
            except Exception as e:
                self.log(f"STOP failed: {e}")

    # ---------- Motion Recorder & File Manager
    def toggle_record(self):
        if not self._is_recording:
            try:
                current = self.read_angles()
            except ValueError as e:
                messagebox.showerror("Cannot record", str(e))
                return
            self.recording = [(0.0, list(current))]
            self._record_start_time = time.time()
            self._is_recording = True
            self.rec_btn.configure(text="■ Stop", bg="#555555")
            self.rec_status_var.set("Recording: 1 pts (0.0s) - Move sliders...")
            self.log("Recording started. Move sliders to teach motion.")
        else:
            self._is_recording = False
            self.rec_btn.configure(text="● Record", bg="#c9302c")
            pts = len(self.recording)
            self.rec_status_var.set(f"Recorded {pts} points.")
            if pts > 1:
                filename = f"rec_{time.strftime('%Y%m%d_%H%M%S')}.json"
                path = os.path.join(RECORDINGS_DIR, filename)
                try:
                    with open(path, "w") as f:
                        json.dump(self.recording, f, indent=2)
                    self.log(f"Saved recording to: {filename}")
                    self.refresh_saved_files(select_filename=filename)
                except Exception as e:
                    self.log(f"Auto-save failed: {e}")
            else:
                self.log("Recording discarded (only 1 point).")

    def toggle_play(self):
        if self._is_playing:
            self.stop_playback()
        else:
            self.start_playback()

    def start_playback(self, custom_trajectory=None, return_home_after=False):
        traj = custom_trajectory if custom_trajectory is not None else self.recording
        if not traj or len(traj) < 2:
            messagebox.showinfo("Nothing to play", "Please record or load a motion file first.")
            return
        if not self.link.connected:
            messagebox.showwarning("Not connected", "Connect to robot serial port before playing.")
            return

        self._is_playing = True
        self.play_btn.configure(text="■ Stop", bg="#555555")
        self._stop_playback_event.clear()
        self._playback_thread = threading.Thread(
            target=self._playback_worker, args=(traj, return_home_after), daemon=True
        )
        self._playback_thread.start()

    def stop_playback(self):
        self._stop_playback_event.set()
        self._is_playing = False
        self.play_btn.configure(text="▶ Play", bg="#28a745")

    def _playback_worker(self, traj, return_home_after=False):
        self.log(f"Playing motion ({len(traj)} points)...")
        while not self._stop_playback_event.is_set():
            t_start = time.time()
            for t_rel, angles in traj:
                if self._stop_playback_event.is_set():
                    break
                dt = (time.time() - t_start)
                wait_time = t_rel - dt
                if wait_time > 0:
                    time.sleep(min(wait_time, 0.05))

                for i, a in enumerate(angles):
                    self.angle_vars[i].set(a)

                try:
                    self.link.send(build_command(angles))
                except Exception:
                    pass

            if not self.loop_var.get() or self._stop_playback_event.is_set():
                break
            time.sleep(0.4)

        if return_home_after and not self._stop_playback_event.is_set():
            time.sleep(0.4)
            self.after(0, self.go_home)

        self._is_playing = False
        self.after(0, lambda: self.play_btn.configure(text="▶ Play", bg="#28a745"))
        self.after(0, lambda: self.rec_status_var.set(f"Idle ({len(traj)} points)"))
        self.log("Playback complete.")

    def save_recording_as(self):
        if not self.recording or len(self.recording) < 2:
            messagebox.showinfo("Empty", "No recorded motion to save.")
            return
        path = filedialog.asksaveasfilename(
            initialdir=RECORDINGS_DIR,
            defaultextension=".json",
            filetypes=[("JSON Motion Files", "*.json")],
            title="Save Motion Recording As"
        )
        if not path:
            return
        try:
            with open(path, "w") as f:
                json.dump(self.recording, f, indent=2)
            fname = os.path.basename(path)
            self.log(f"Saved: {fname}")
            self.refresh_saved_files(select_filename=fname)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save: {e}")

    def load_selected_file(self):
        filename = self.selected_file_var.get()
        if not filename:
            return
        path = os.path.join(RECORDINGS_DIR, filename)
        if not os.path.exists(path):
            messagebox.showerror("Missing", f"File does not exist: {filename}")
            return
        try:
            with open(path, "r") as f:
                data = json.load(f)
            self.recording = [(float(pt[0]), [float(a) for a in pt[1]]) for pt in data]
            self.rec_status_var.set(f"Loaded '{filename}' ({len(self.recording)} pts)")
            self.log(f"Loaded: {filename} ({len(self.recording)} points)")
        except Exception as e:
            messagebox.showerror("Load Failed", f"Could not parse file: {e}")

    def delete_selected_file(self):
        filename = self.selected_file_var.get()
        if not filename:
            return
        if messagebox.askyesno("Delete", f"Delete '{filename}' permanently?"):
            try:
                os.remove(os.path.join(RECORDINGS_DIR, filename))
                self.log(f"Deleted {filename}")
                self.refresh_saved_files()
            except Exception as e:
                messagebox.showerror("Delete Failed", str(e))

    def on_file_selected(self, _event=None):
        pass

    def clear_recording(self):
        self.recording = []
        self.rec_status_var.set("Recorder: Idle (0 points)")
        self.log("Cleared recording buffer.")

    def open_recordings_folder(self):
        try:
            os.startfile(RECORDINGS_DIR)
        except Exception:
            self.log(f"Recordings folder: {RECORDINGS_DIR}")

    # ---------- Camera & Zero-Lag Feed
    def refresh_camera_list(self):
        if not CV_AVAILABLE:
            return
        cam_options = []
        for idx in [2, 1, 0, 3]:
            backend = cv2.CAP_DSHOW if os.name == 'nt' else 0
            cap = cv2.VideoCapture(idx, backend)
            if cap.isOpened():
                tag = "USB Webcam" if idx > 0 else "Laptop Integrated"
                cam_options.append((idx, f"Camera {idx} ({tag})"))
                cap.release()

        if not cam_options:
            cam_options = [(2, "Camera 2 (USB Webcam)"), (1, "Camera 1 (USB Webcam)"), (0, "Camera 0 (Laptop Integrated)")]

        self.cam_combo["values"] = [lbl for _, lbl in cam_options]
        self._cam_map = {lbl: idx for idx, lbl in cam_options}
        self.cam_combo.set(cam_options[0][1])
        self.active_cam_idx = cam_options[0][0]

    def on_camera_selected(self, _event=None):
        lbl = self.cam_combo.get()
        if lbl in self._cam_map:
            self.active_cam_idx = self._cam_map[lbl]
            self.log(f"Selected {lbl}...")
            self.start_camera()

    def start_camera(self):
        if not CV_AVAILABLE:
            return
        ok = self.cam.start(self.active_cam_idx)
        if ok:
            self.cam_toggle_btn.configure(text="Stop Cam")
            self.log(f"Camera {self.active_cam_idx} started smoothly (Zero Lag).")
        else:
            self.log(f"Camera {self.active_cam_idx} not responding. Trying other indices...")
            for alt in [1, 0, 3]:
                if alt != self.active_cam_idx and self.cam.start(alt):
                    self.active_cam_idx = alt
                    self.cam_toggle_btn.configure(text="Stop Cam")
                    self.log(f"Connected to alternate Camera {alt}!")
                    break

    def stop_camera(self):
        if self.cam:
            self.cam.stop()
        self.cam_toggle_btn.configure(text="Start Cam")
        self.video_canvas.delete("all")
        self.canvas_img_id = self.video_canvas.create_image(0, 0, anchor=tk.NW)
        self.video_canvas.create_text(190, 140, text="Webcam stopped.", fill="#888888")

    def toggle_camera(self):
        if self.cam and self.cam.is_running:
            self.stop_camera()
        else:
            self.start_camera()

    def _update_camera_feed(self):
        """Zero-lag canvas updater: retrieves latest frame from background thread."""
        if self.cam and self.cam.is_running:
            ret, frame = self.cam.get_latest_frame()
            if ret and frame is not None:
                frame_resized = cv2.resize(frame, (380, 280))
                h, w = frame_resized.shape[:2]

                # Center Pick Zone (120x120)
                rw, rh = 120, 120
                rx1, ry1 = (w - rw) // 2, (h - rh) // 2
                rx2, ry2 = rx1 + rw, ry1 + rh
                roi = frame_resized[ry1:ry2, rx1:rx2]

                # Detect Color
                color, conf = self.detector.detect(roi)
                self.current_detected_color = color
                self.color_confidence = conf

                # Draw Target Box
                box_color = (0, 255, 0) if color != "NONE" else (180, 180, 180)
                cv2.rectangle(frame_resized, (rx1, ry1), (rx2, ry2), box_color, 2)
                cv2.putText(frame_resized, "PICK ZONE", (rx1, ry1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA)

                # Update Color Badge
                if color != "NONE":
                    self.color_badge.configure(
                        text=f"● DETECTED: {color} ({int(conf * 100)}%)",
                        bg=COLOR_HEX[color]
                    )
                else:
                    self.color_badge.configure(text="NO PRODUCT DETECTED", bg="#555555")

                # Handle Auto-Sorting State Machine
                self._handle_auto_sort(color)

                # Render to Tkinter Canvas with Item Re-use (No Canvas Object Leak)
                rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)
                self.photo = ImageTk.PhotoImage(image=img)
                self.video_canvas.itemconfig(self.canvas_img_id, image=self.photo)

        # Refresh at a steady 33ms (~30 FPS) with zero main-thread block
        self.after(33, self._update_camera_feed)

    # ---------- 1-Click Color Assign & Auto-Sorting
    def quick_assign_to_color(self, color_name: str):
        if not self.recording or len(self.recording) < 2:
            messagebox.showinfo("No Recording", "Move sliders to teach a motion first, then click assign.")
            return

        filename = f"rec_{color_name.lower()}.json"
        path = os.path.join(RECORDINGS_DIR, filename)
        try:
            with open(path, "w") as f:
                json.dump(self.recording, f, indent=2)
            self.color_mappings[color_name].set(filename)
            self.save_mappings()
            self.refresh_saved_files(select_filename=filename)
            self.log(f"Assigned current motion to {color_name} ({len(self.recording)} points)!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to assign: {e}")

    def load_saved_mappings(self):
        # 1. Load from vision_mapping.json if exists
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r") as f:
                    data = json.load(f)
                    for c in COLORS:
                        if c in data and data[c]:
                            self.color_mappings[c].set(data[c])
            except Exception:
                pass

        # 2. Automatically link rec_<color>.json files from recordings folder
        try:
            available_files = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith(".json")]
            for c in COLORS:
                current_val = self.color_mappings[c].get()
                if not current_val or current_val == "(None)":
                    candidate = f"rec_{c.lower()}.json"
                    if candidate in available_files:
                        self.color_mappings[c].set(candidate)
                    else:
                        for f in available_files:
                            if c.lower() in f.lower():
                                self.color_mappings[c].set(f)
                                break
        except Exception:
            pass

    def save_mappings(self):
        try:
            data = {c: self.color_mappings[c].get() for c in COLORS}
            with open(CONFIG_FILE, "w") as f:
                json.dump(data, f, indent=2)
            self.log("Saved color-to-trajectory mappings.")
        except Exception as e:
            self.log(f"Mapping save failed: {e}")

    def test_playback_color(self, color_name: str):
        fname = self.color_mappings[color_name].get()
        if not fname or fname == "(None)":
            messagebox.showinfo("No Mapping", f"No recording assigned to {color_name} yet.")
            return
        path = os.path.join(RECORDINGS_DIR, fname)
        if not os.path.exists(path):
            messagebox.showerror("Missing", f"File not found: {fname}")
            return
        try:
            with open(path, "r") as f:
                data = json.load(f)
            traj = [(float(pt[0]), [float(a) for a in pt[1]]) for pt in data]
            self.log(f"Testing playback for {color_name} ({fname})...")
            self.start_playback(custom_trajectory=traj, return_home_after=True)
        except Exception as e:
            messagebox.showerror("Error", f"Could not play {fname}: {e}")

    def toggle_auto_sort(self):
        self.auto_sort_active = not self.auto_sort_active
        if self.auto_sort_active:
            self.auto_btn.configure(text="■ Pause Auto-Sorting", bg="#c9302c")
            self.auto_status_var.set("Auto-Sort: ACTIVE (Place product in Pick Zone)")
            self.log("Auto-Sorting Mode ACTIVE.")
        else:
            self.auto_btn.configure(text="🔄 Start Auto-Sorting Mode", bg="#28a745")
            self.auto_status_var.set("Auto-Sort: Paused")
            self.log("Auto-Sorting Mode paused.")

    def trigger_manual_sort(self):
        if self.current_detected_color != "NONE":
            self._execute_color_sort(self.current_detected_color)
        else:
            messagebox.showinfo("No Product", "No product detected in the center Pick Zone.")

    def _handle_auto_sort(self, color: str):
        if not self.auto_sort_active or self._is_playing:
            return

        if time.time() - self.last_sorted_time < 2.5:
            return  # Cooldown between sorts

        if color != "NONE":
            if color == self.last_seen_color:
                self.stable_color_counter += 1
                if self.stable_color_counter >= 8:  # ~0.25s of stable detection
                    self.stable_color_counter = 0
                    self._execute_color_sort(color)
            else:
                self.last_seen_color = color
                self.stable_color_counter = 0
        else:
            self.stable_color_counter = 0
            self.last_seen_color = "NONE"

    def _execute_color_sort(self, color_name: str):
        fname = self.color_mappings[color_name].get()
        # Fallback auto-search in recordings directory
        if not fname or fname == "(None)":
            candidate = f"rec_{color_name.lower()}.json"
            if os.path.exists(os.path.join(RECORDINGS_DIR, candidate)):
                fname = candidate
                self.color_mappings[color_name].set(fname)
            else:
                for f in os.listdir(RECORDINGS_DIR):
                    if color_name.lower() in f.lower() and f.endswith(".json"):
                        fname = f
                        self.color_mappings[color_name].set(fname)
                        break

        if not fname or fname == "(None)":
            self.color_badge.configure(
                text=f"● {color_name} DETECTED (No motion file mapped!)",
                bg=COLOR_HEX[color_name]
            )
            self.log(f"Detected {color_name}, but no recording is mapped. Teach motion and click [→ {color_name}]!")
            return

        path = os.path.join(RECORDINGS_DIR, fname)
        if not os.path.exists(path):
            self.log(f"Motion file missing: {fname}")
            return

        try:
            with open(path, "r") as f:
                data = json.load(f)
            traj = [(float(pt[0]), [float(a) for a in pt[1]]) for pt in data]
            self.color_badge.configure(
                text=f"● DETECTED: {color_name} -> AUTO-STARTING '{fname}'!",
                bg=COLOR_HEX[color_name]
            )
            self.log(f"🎯 Color '{color_name}' detected! Automatically starting '{fname}' ({len(traj)} points)...")
            self.start_playback(custom_trajectory=traj, return_home_after=True)
            self.last_sorted_time = time.time()
        except Exception as e:
            self.log(f"Auto-start failed: {e}")

    # ---------- RX Queue & Shutdown
    def _poll_rx(self):
        try:
            while True:
                kind, msg = self.rx_queue.get_nowait()
                if kind == "rx":
                    self.log(f"RX: {msg}")
                elif kind == "error":
                    self.log(msg)
                    self.link.disconnect()
                    self.connect_btn.configure(text="Connect")
                    self.status_var.set("Disconnected (error)")
        except queue.Empty:
            pass
        self.after(50, self._poll_rx)

    def on_close(self):
        self.stop_playback()
        if self.cam:
            self.cam.stop()
        self.link.disconnect()
        self.destroy()


if __name__ == "__main__":
    VisionArmGUI().mainloop()
