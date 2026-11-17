"""
6-Axis Arm Controller GUI with Motion Auto-Save & Trajectory Recorder
---------------------------------------------------------------------
Requirements:  pip install pyserial
Run:           python axis.py

Safe Home Angles:
  J1 = 90.0°  (Base: 0° to 180°)
  J2 = 80.0°  (Shoulder: 0° to 180°)
  J3 = 108.0° (Elbow: 0° to 180°)
  J4 = 38.0°  (Wrist Pitch: 0° to 180°)
  J5 = 96.0°  (Wrist Roll: 0° to 180°)
  J6 = 45.0°  (Gripper: 40° to 100° Locked Range)
"""

import json
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------- configuration
NUM_JOINTS = 6

# Individual Joint Limits: (min_deg, max_deg)
# J6 (Gripper) is restricted to 40° to 100° to prevent mechanism jamming
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
LIVE_DEBOUNCE_MS = 25                        # ~40 Hz refresh for smooth live tracking

# Directory for storing recorded trajectory files
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RECORDINGS_DIR = os.path.join(SCRIPT_DIR, "recordings")
os.makedirs(RECORDINGS_DIR, exist_ok=True)


def build_command(angles):
    """Format joint angles to the serial command protocol."""
    return "MOVE " + ",".join(f"{a:.2f}" for a in angles) + "\n"


# ---------------------------------------------------------------- serial layer
class SerialLink:
    """Owns the serial port and a background reader thread."""

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
        # Allow microcontroller to complete auto-reset
        time.sleep(2.0)
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
            except (serial.SerialException, OSError) as exc:
                self.rx_queue.put(("error", f"Serial error: {exc}"))
                return
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.rx_queue.put(("rx", line.decode(errors="replace").strip()))


# ---------------------------------------------------------------- GUI
class ArmGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("6-Axis Arm Controller (J6 Home: 45° | 40°–100° Range)")
        self.resizable(False, False)

        self.rx_queue = queue.Queue()
        self.link = SerialLink(self.rx_queue)
        self.angle_vars: list[tk.DoubleVar] = []
        self._live_job = None

        # Recorder & Playback State
        self.recording: list[tuple[float, list[float]]] = []  # [(relative_time, [j1..j6]), ...]
        self._is_recording = False
        self._record_start_time = 0.0
        self._is_playing = False
        self._playback_thread = None
        self._stop_playback_event = threading.Event()

        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.rec_status_var = tk.StringVar(value="Recorder: Idle (0 points)")
        self.loop_var = tk.BooleanVar(value=False)
        self.speed_var = tk.IntVar(value=65)
        self.selected_file_var = tk.StringVar()

        self._build_ui()
        self.refresh_ports()
        self.refresh_saved_files()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(50, self._poll_rx)

    # ---------- UI construction
    def _build_ui(self):
        pad = {"padx": 5, "pady": 3}

        # 1. Connection bar
        conn = ttk.LabelFrame(self, text="Connection")
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

        # 2. Joint sliders
        joints = ttk.LabelFrame(self, text="Joint Angles (Live Control)")
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

        # 3. Motion Recorder & Playback Panel (With Auto-Save & File Manager)
        rec_frame = ttk.LabelFrame(self, text="Motion Recorder & Saved Files")
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

        # 4. Commands & Smoothness / Speed Control
        cmd = ttk.LabelFrame(self, text="PID Speed & Commands")
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

        # 5. Log
        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.grid(row=4, column=0, sticky="nsew", padx=8, pady=(4, 8))
        self.log_text = tk.Text(log_frame, height=8, width=72, state="disabled")
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")

    # ---------- helpers
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
        """Scan the recordings folder and update the dropdown."""
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
        except Exception as e:
            self.log(f"Error refreshing recordings: {e}")

    def read_angles(self) -> list[float]:
        """Read and validate all joint values against limits."""
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

    # ---------- connection
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

    # ---------- live slider movement (debounced)
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
                self.rec_status_var.set(f"● RECORDING: {len(self.recording)} pts ({t:.1f}s)")
        except (ConnectionError, serial.SerialException, OSError) as exc:
            self.log(f"Send failed: {exc}")

    # ---------- Recorder Logic with Automatic Saving
    def toggle_record(self):
        if self._is_playing:
            messagebox.showwarning("Busy", "Stop playback before recording.")
            return

        if not self._is_recording:
            try:
                cur_angles = self.read_angles()
            except ValueError as e:
                messagebox.showerror("Error", str(e))
                return

            self.recording = [(0.0, list(cur_angles))]
            self._record_start_time = time.time()
            self._is_recording = True

            self.rec_btn.configure(text="■ Stop Record", bg="#555555")
            self.play_btn.configure(state="disabled")
            self.rec_status_var.set("● RECORDING: Move the sliders now...")
            self.log("Started recording arm movements.")
        else:
            self._is_recording = False
            self.rec_btn.configure(text="● Record", bg="#c9302c")
            self.play_btn.configure(state="normal")

            dur = self.recording[-1][0] if self.recording else 0.0

            saved_file = self.auto_save_recording()
            if saved_file:
                self.rec_status_var.set(f"✓ Saved: {saved_file} ({len(self.recording)} pts, {dur:.1f}s)")
                self.log(f"Auto-saved recording to '{saved_file}' ({len(self.recording)} pts, {dur:.1f}s)")
            else:
                self.rec_status_var.set(f"Recorded: {len(self.recording)} points ({dur:.1f}s)")

    def auto_save_recording(self):
        """Automatically saves the recording with a timestamp."""
        if not self.recording or len(self.recording) < 2:
            return None
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"motion_{timestamp}.json"
        filepath = os.path.join(RECORDINGS_DIR, filename)
        try:
            with open(filepath, "w") as f:
                json.dump(self.recording, f, indent=2)
            self.refresh_saved_files(select_filename=filename)
            return filename
        except Exception as e:
            self.log(f"Auto-save failed: {e}")
            return None

    def save_recording_as(self):
        """Allows user to save the recording with a custom name."""
        if not self.recording:
            messagebox.showinfo("Empty", "No recorded movements to save.")
            return
        path = filedialog.asksaveasfilename(
            initialdir=RECORDINGS_DIR,
            defaultextension=".json",
            filetypes=[("JSON Motion Files", "*.json"), ("All Files", "*.*")]
        )
        if path:
            try:
                with open(path, "w") as f:
                    json.dump(self.recording, f, indent=2)
                fname = os.path.basename(path)
                self.refresh_saved_files(select_filename=fname)
                self.log(f"Saved recording as {fname}")
                messagebox.showinfo("Saved", f"Recording saved successfully as:\n{fname}")
            except Exception as e:
                messagebox.showerror("Save Error", str(e))

    def on_file_selected(self, _event=None):
        self.load_selected_file()

    def load_selected_file(self):
        """Load selected file from recordings dropdown."""
        if self._is_recording or self._is_playing:
            messagebox.showwarning("Busy", "Cannot load while recording or playing.")
            return
        filename = self.selected_file_var.get()
        if not filename:
            messagebox.showinfo("Select File", "Please select a recording file from the dropdown.")
            return
        filepath = os.path.join(RECORDINGS_DIR, filename)
        if not os.path.exists(filepath):
            messagebox.showerror("Not Found", f"File '{filename}' does not exist.")
            return
        try:
            with open(filepath, "r") as f:
                data = json.load(f)
            if isinstance(data, list) and len(data) > 0 and len(data[0]) == 2:
                cleaned_recording = []
                for t, angles in data:
                    clamped_angles = []
                    for i, a in enumerate(angles):
                        lo, hi = JOINT_LIMITS[i]
                        clamped_angles.append(min(max(float(a), lo), hi))
                    cleaned_recording.append((float(t), clamped_angles))
                self.recording = cleaned_recording
                dur = self.recording[-1][0]
                self.rec_status_var.set(f"Loaded: {filename} ({len(self.recording)} pts, {dur:.1f}s)")
                self.log(f"Loaded '{filename}' ({len(self.recording)} points, {dur:.1f}s)")
            else:
                messagebox.showerror("Invalid File", "File format is not a valid trajectory.")
        except Exception as e:
            messagebox.showerror("Load Error", str(e))

    def delete_selected_file(self):
        """Deletes the currently selected recording file."""
        filename = self.selected_file_var.get()
        if not filename:
            return
        if messagebox.askyesno("Confirm Delete", f"Are you sure you want to delete:\n{filename}?"):
            try:
                filepath = os.path.join(RECORDINGS_DIR, filename)
                if os.path.exists(filepath):
                    os.remove(filepath)
                    self.log(f"Deleted {filename}")
                    self.refresh_saved_files()
                    self.clear_recording()
            except Exception as e:
                messagebox.showerror("Delete Error", str(e))

    def open_recordings_folder(self):
        """Opens the recordings directory in Windows File Explorer."""
        try:
            os.startfile(RECORDINGS_DIR)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def clear_recording(self):
        if self._is_recording or self._is_playing:
            messagebox.showwarning("Busy", "Cannot clear while recording or playing.")
            return
        self.recording = []
        self.rec_status_var.set("Recorder: Idle (0 points)")
        self.log("Recording cleared from memory.")

    # ---------- Playback Logic
    def toggle_play(self):
        if self._is_playing:
            self.stop_playback()
        else:
            self.start_playback()

    def start_playback(self):
        if not self.link.connected:
            messagebox.showwarning("Not connected", "Connect to the arm serial port first.")
            return
        if not self.recording:
            if self.selected_file_var.get():
                self.load_selected_file()
            if not self.recording:
                messagebox.showinfo("Empty", "No recorded movements found. Record or select a file first.")
                return

        self._is_playing = True
        self._stop_playback_event.clear()
        self.play_btn.configure(text="■ Stop Play", bg="#555555")
        self.rec_btn.configure(state="disabled")

        self.log(f"Starting playback ({len(self.recording)} points)...")
        self._playback_thread = threading.Thread(target=self._playback_worker, daemon=True)
        self._playback_thread.start()

    def stop_playback(self):
        if not self._is_playing:
            return
        self._stop_playback_event.set()
        self._is_playing = False
        self.play_btn.configure(text="▶ Play", bg="#28a745")
        self.rec_btn.configure(state="normal")
        self.rec_status_var.set("Playback stopped.")
        self.log("Playback stopped.")

    def _playback_worker(self):
        while self._is_playing and not self._stop_playback_event.is_set():
            start_wall = time.time()
            total_pts = len(self.recording)

            for idx, (target_t, angles) in enumerate(self.recording):
                if self._stop_playback_event.is_set() or not self._is_playing:
                    break

                elapsed = time.time() - start_wall
                wait_needed = target_t - elapsed
                if wait_needed > 0.002:
                    time.sleep(wait_needed)

                if self._stop_playback_event.is_set() or not self._is_playing:
                    break

                try:
                    self.link.send(build_command(angles))
                except Exception as e:
                    self.rx_queue.put(("error", f"Playback send failed: {e}"))
                    break

                self.after(0, self._playback_tick_gui, angles, idx + 1, total_pts)

            if not self.loop_var.get() or self._stop_playback_event.is_set() or not self._is_playing:
                break
            time.sleep(0.3)

        self.after(0, self._finish_playback)

    def _playback_tick_gui(self, angles: list[float], current_idx: int, total_pts: int):
        for i, val in enumerate(angles):
            if i < len(self.angle_vars):
                self.angle_vars[i].set(val)

        pct = int((current_idx / total_pts) * 100)
        self.rec_status_var.set(f"▶ PLAYING: {current_idx}/{total_pts} ({pct}%)")

    def _finish_playback(self):
        if self._is_playing:
            self._is_playing = False
            self.play_btn.configure(text="▶ Play", bg="#28a745")
            self.rec_btn.configure(state="normal")
            dur = self.recording[-1][0] if self.recording else 0.0
            self.rec_status_var.set(f"Playback complete ({len(self.recording)} pts, {dur:.1f}s)")
            self.log("Playback completed successfully.")

    # ---------- Commands
    def go_home(self):
        """Move all 6 joints smoothly to Safe Home position."""
        if self._is_playing:
            self.stop_playback()
        for i, var in enumerate(self.angle_vars):
            var.set(HOME_POSITION[i])
        if self.link.connected:
            try:
                self.link.send("HOME\n")
                self.log(f"TX: HOME -> Safe Home Position {HOME_POSITION}")
            except (ConnectionError, serial.SerialException, OSError) as exc:
                self.log(f"Home send failed: {exc}")

    def emergency_stop(self):
        """Emergency software stop."""
        self.stop_playback()
        if self._is_recording:
            self.toggle_record()

        if self._live_job is not None:
            self.after_cancel(self._live_job)
            self._live_job = None
        if self.link.connected:
            try:
                self.link.send("STOP\n")
                self.log("TX: STOP (Emergency Stop Activated)")
            except (ConnectionError, serial.SerialException, OSError) as exc:
                self.log(f"STOP failed: {exc}")

    # ---------- receive loop
    def _poll_rx(self):
        try:
            while True:
                kind, msg = self.rx_queue.get_nowait()
                if kind == "rx":
                    self.log(f"RX: {msg}")
                elif kind == "error":
                    self.log(msg)
                    self.stop_playback()
                    self.link.disconnect()
                    self.connect_btn.configure(text="Connect")
                    self.status_var.set("Disconnected (error)")
        except queue.Empty:
            pass
        self.after(50, self._poll_rx)

    # ---------- shutdown
    def on_close(self):
        self.stop_playback()
        self.link.disconnect()
        self.destroy()


if __name__ == "__main__":
    ArmGUI().mainloop()
