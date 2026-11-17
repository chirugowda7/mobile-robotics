#!/usr/bin/env python3
"""
Web UI Server for Camera Line Follower Robot.
Provides a responsive browser-based interface with:
  - START / STOP / PAUSE controls
  - Color selection buttons (Yellow, Green, Blue)
  - Live camera MJPEG video stream
  - Real-time telemetry overlay
  - Speed adjustment slider

Access from any browser on the network: http://<pi-ip>:5000
"""

import sys
import time
import json
import signal
import threading
import argparse
from io import BytesIO

import cv2
import numpy as np

try:
    from flask import Flask, render_template_string, Response, request, jsonify
except ImportError:
    print("[ERROR] Flask is not installed. Install with: pip3 install flask")
    sys.exit(1)

import config
from motor_driver import CytronMotorDriver
from line_detector import LineDetector
from pid_controller import PIDController

# ==============================================================================
# Global Robot State
# ==============================================================================
class RobotState:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = False          # Main follower loop active
        self.paused = False           # Motors paused but camera still running
        self.active_color = "yellow"
        self.base_speed = config.BASE_SPEED
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.steering = 0.0
        self.error = 0.0
        self.detected = False
        self.fps = 0.0
        self.frames_lost = 0
        self.motor_mode = "simulation"  # or "hardware"

    def to_dict(self):
        with self.lock:
            return {
                "running": self.running,
                "paused": self.paused,
                "active_color": self.active_color,
                "base_speed": self.base_speed,
                "left_speed": round(self.left_speed, 1),
                "right_speed": round(self.right_speed, 1),
                "steering": round(self.steering, 2),
                "error": round(self.error, 3),
                "detected": self.detected,
                "fps": round(self.fps, 1),
                "frames_lost": self.frames_lost,
                "motor_mode": self.motor_mode,
            }

state = RobotState()
latest_frame = None
frame_lock = threading.Lock()

# ==============================================================================
# Robot Control Thread
# ==============================================================================
class RobotController(threading.Thread):
    def __init__(self, camera_index=config.CAMERA_INDEX):
        super().__init__(daemon=True)
        self.camera_index = camera_index
        self.detector = LineDetector(initial_color=state.active_color)
        self.pid = PIDController()
        self.motors = CytronMotorDriver()
        self.cap = None
        self._stop_event = threading.Event()

        state.motor_mode = "hardware" if self.motors.is_hardware_active else "simulation"

    def open_camera(self):
        if self.cap and self.cap.isOpened():
            return True
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            for alt in [0, 1, 2]:
                if alt != self.camera_index:
                    self.cap = cv2.VideoCapture(alt)
                    if self.cap.isOpened():
                        break
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
            return True
        return False

    def run(self):
        global latest_frame
        if not self.open_camera():
            print("[ROBOT] FATAL: Cannot open camera.")
            return

        fps_timer = time.time()
        frame_count = 0
        last_detection_time = time.time()

        print("[ROBOT] Camera opened. Control thread running.")
        while not self._stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # Sync color from UI
            with state.lock:
                desired_color = state.active_color
                is_running = state.running
                is_paused = state.paused
                base_speed = state.base_speed

            if self.detector.get_active_color() != desired_color:
                self.detector.set_color(desired_color)
                self.pid.reset()

            # Always process frame for the video feed
            result = self.detector.process_frame(frame)
            detected = result["detected"]
            error = result["error"]

            # Motor control only when running and not paused
            left_spd = 0.0
            right_spd = 0.0
            steering = 0.0

            if is_running and not is_paused:
                if detected:
                    last_detection_time = time.time()
                    steering = self.pid.update(error)
                    left_spd, right_spd = config.compute_wheel_speeds(base_speed, steering, error)
                    self.motors.set_speeds(left_spd, right_spd)
                else:
                    lost_duration = time.time() - last_detection_time
                    if lost_duration < config.LOST_LINE_TIMEOUT:
                        # Recovery: actively pivot in the direction line was last seen
                        pivot_speed = base_speed * 0.75
                        if error > 0:
                            # Last seen on right -> pivot right (Left forward, Right reverse)
                            left_spd = pivot_speed
                            right_spd = -pivot_speed
                        else:
                            # Last seen on left -> pivot left (Left reverse, Right forward)
                            left_spd = -pivot_speed
                            right_spd = pivot_speed
                        self.motors.set_speeds(left_spd, right_spd)
                    else:
                        self.motors.stop()
                        self.pid.reset()
            else:
                self.motors.stop()

            # FPS
            frame_count += 1
            elapsed = time.time() - fps_timer
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_timer = time.time()
                with state.lock:
                    state.fps = fps

            # Update shared state
            with state.lock:
                state.left_speed = left_spd
                state.right_speed = right_spd
                state.steering = steering
                state.error = error
                state.detected = detected
                state.frames_lost = result["frames_lost"]

            # Draw HUD on annotated frame for the stream
            annotated = result["annotated"]
            self._draw_stream_hud(annotated, result, left_spd, right_spd, steering, is_running, is_paused)

            # Encode for MJPEG stream
            _, jpeg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with frame_lock:
                latest_frame = jpeg.tobytes()

        # Cleanup
        self.motors.stop()
        self.motors.cleanup()
        if self.cap:
            self.cap.release()
        print("[ROBOT] Control thread stopped.")

    def _draw_stream_hud(self, frame, result, left_spd, right_spd, steering, is_running, is_paused):
        h, w = frame.shape[:2]
        # Top banner
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 40), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

        color_name = self.detector.get_active_color().upper()
        with state.lock:
            fps = state.fps

        if not is_running:
            status = "STOPPED"
            col = (128, 128, 128)
        elif is_paused:
            status = "PAUSED"
            col = (0, 165, 255)
        elif result["detected"]:
            status = "TRACKING"
            col = (0, 255, 0)
        else:
            status = "SEARCHING"
            col = (0, 215, 255)

        cv2.putText(frame, f"[{color_name}] {status}", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.putText(frame, f"FPS:{fps:.0f} L:{left_spd:.0f}% R:{right_spd:.0f}%", (w - 260, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

    def stop(self):
        self._stop_event.set()


# ==============================================================================
# Flask Web Application
# ==============================================================================
app = Flask(__name__)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Line Follower Robot</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
            background: #0a0e17;
            color: #e0e0e0;
            min-height: 100vh;
            overflow-x: hidden;
        }

        /* Header */
        .header {
            background: linear-gradient(135deg, #1a1f35 0%, #0d1220 100%);
            padding: 14px 24px;
            display: flex;
            align-items: center;
            justify-content: space-between;
            border-bottom: 2px solid #2a3a5c;
            box-shadow: 0 4px 20px rgba(0,0,0,0.4);
        }
        .header h1 {
            font-size: 1.3rem;
            font-weight: 700;
            background: linear-gradient(90deg, #00d4ff, #7b68ee);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            letter-spacing: 1px;
        }
        .header .status-badge {
            padding: 6px 18px;
            border-radius: 20px;
            font-size: 0.8rem;
            font-weight: 700;
            letter-spacing: 1.5px;
            text-transform: uppercase;
        }
        .status-stopped { background: #333; color: #888; border: 1px solid #555; }
        .status-running { background: rgba(0,220,100,0.15); color: #00dc64; border: 1px solid #00dc64; animation: pulse-glow 2s infinite; }
        .status-paused  { background: rgba(255,165,0,0.15); color: #ffa500; border: 1px solid #ffa500; }

        @keyframes pulse-glow {
            0%, 100% { box-shadow: 0 0 5px rgba(0,220,100,0.3); }
            50% { box-shadow: 0 0 20px rgba(0,220,100,0.6); }
        }

        /* Main Grid */
        .main-grid {
            display: grid;
            grid-template-columns: 1fr 320px;
            gap: 16px;
            padding: 16px;
            max-width: 1400px;
            margin: 0 auto;
        }
        @media (max-width: 900px) {
            .main-grid { grid-template-columns: 1fr; }
        }

        /* Video Feed */
        .video-container {
            background: #111827;
            border-radius: 16px;
            overflow: hidden;
            border: 2px solid #1e293b;
            box-shadow: 0 8px 32px rgba(0,0,0,0.5);
            position: relative;
        }
        .video-container img {
            width: 100%;
            display: block;
            min-height: 360px;
            object-fit: contain;
            background: #000;
        }
        .video-offline {
            display: flex;
            align-items: center;
            justify-content: center;
            min-height: 400px;
            color: #555;
            font-size: 1.2rem;
        }
        .video-offline svg { margin-right: 10px; }

        /* Control Panel */
        .control-panel {
            display: flex;
            flex-direction: column;
            gap: 14px;
        }
        .card {
            background: linear-gradient(145deg, #151c2e 0%, #111827 100%);
            border-radius: 14px;
            padding: 18px;
            border: 1px solid #1e293b;
            box-shadow: 0 4px 16px rgba(0,0,0,0.3);
        }
        .card-title {
            font-size: 0.7rem;
            text-transform: uppercase;
            letter-spacing: 2px;
            color: #64748b;
            margin-bottom: 14px;
            font-weight: 600;
        }

        /* Start/Stop Buttons */
        .action-buttons {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }
        .btn {
            padding: 14px;
            border: none;
            border-radius: 12px;
            font-size: 0.95rem;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.2s ease;
            letter-spacing: 1px;
            text-transform: uppercase;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
        }
        .btn:active { transform: scale(0.96); }

        .btn-start {
            background: linear-gradient(135deg, #059669, #10b981);
            color: #fff;
            box-shadow: 0 4px 15px rgba(5,150,105,0.4);
        }
        .btn-start:hover { background: linear-gradient(135deg, #047857, #059669); }
        .btn-start:disabled { opacity: 0.4; cursor: not-allowed; }

        .btn-stop {
            background: linear-gradient(135deg, #dc2626, #ef4444);
            color: #fff;
            box-shadow: 0 4px 15px rgba(220,38,38,0.4);
        }
        .btn-stop:hover { background: linear-gradient(135deg, #b91c1c, #dc2626); }
        .btn-stop:disabled { opacity: 0.4; cursor: not-allowed; }

        .btn-pause {
            grid-column: span 2;
            background: linear-gradient(135deg, #d97706, #f59e0b);
            color: #000;
            box-shadow: 0 4px 15px rgba(217,119,6,0.3);
        }
        .btn-pause:hover { background: linear-gradient(135deg, #b45309, #d97706); }
        .btn-pause:disabled { opacity: 0.4; cursor: not-allowed; }

        /* Color Selection */
        .color-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
        }
        .color-btn {
            padding: 14px 8px;
            border: 3px solid transparent;
            border-radius: 12px;
            font-size: 0.85rem;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.2s ease;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .color-btn:active { transform: scale(0.95); }

        .color-yellow {
            background: rgba(234,179,8,0.15);
            color: #facc15;
            border-color: #44400a;
        }
        .color-yellow.active, .color-yellow:hover {
            background: rgba(234,179,8,0.35);
            border-color: #facc15;
            box-shadow: 0 0 20px rgba(250,204,21,0.3);
        }
        .color-green {
            background: rgba(34,197,94,0.12);
            color: #4ade80;
            border-color: #14412a;
        }
        .color-green.active, .color-green:hover {
            background: rgba(34,197,94,0.3);
            border-color: #4ade80;
            box-shadow: 0 0 20px rgba(74,222,128,0.3);
        }
        .color-blue {
            background: rgba(59,130,246,0.12);
            color: #60a5fa;
            border-color: #1e3a5f;
        }
        .color-blue.active, .color-blue:hover {
            background: rgba(59,130,246,0.3);
            border-color: #60a5fa;
            box-shadow: 0 0 20px rgba(96,165,250,0.3);
        }

        /* Speed Slider */
        .speed-row {
            display: flex;
            align-items: center;
            gap: 12px;
        }
        .speed-row input[type=range] {
            flex: 1;
            accent-color: #7b68ee;
            height: 6px;
        }
        .speed-value {
            font-size: 1.5rem;
            font-weight: 800;
            color: #7b68ee;
            min-width: 55px;
            text-align: right;
        }

        /* Telemetry */
        .telemetry-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }
        .telem-item {
            background: rgba(0,0,0,0.3);
            border-radius: 10px;
            padding: 10px 12px;
            text-align: center;
        }
        .telem-label {
            font-size: 0.6rem;
            text-transform: uppercase;
            letter-spacing: 1.5px;
            color: #64748b;
            margin-bottom: 4px;
        }
        .telem-value {
            font-size: 1.2rem;
            font-weight: 800;
            font-family: 'SF Mono', 'Cascadia Code', monospace;
        }
        .telem-value.tracking { color: #4ade80; }
        .telem-value.lost { color: #f87171; }

        /* Motor Bars */
        .motor-bars {
            display: flex;
            gap: 12px;
            align-items: flex-end;
        }
        .motor-bar-group {
            flex: 1;
            text-align: center;
        }
        .motor-bar-label {
            font-size: 0.65rem;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: #64748b;
            margin-bottom: 6px;
        }
        .motor-bar-track {
            height: 100px;
            background: rgba(0,0,0,0.4);
            border-radius: 8px;
            position: relative;
            overflow: hidden;
        }
        .motor-bar-fill {
            position: absolute;
            bottom: 0;
            left: 0;
            right: 0;
            background: linear-gradient(to top, #7b68ee, #a78bfa);
            border-radius: 0 0 8px 8px;
            transition: height 0.15s ease;
        }
        .motor-bar-value {
            margin-top: 4px;
            font-size: 0.9rem;
            font-weight: 700;
            font-family: monospace;
            color: #a78bfa;
        }

        /* Connection indicator */
        .conn-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            display: inline-block;
            margin-right: 6px;
        }
        .conn-online { background: #4ade80; box-shadow: 0 0 8px #4ade80; }
        .conn-offline { background: #f87171; }
    </style>
</head>
<body>
    <!-- Header -->
    <header class="header">
        <h1>🤖 LINE FOLLOWER ROBOT</h1>
        <div class="status-badge status-stopped" id="statusBadge">STOPPED</div>
    </header>

    <!-- Main Layout -->
    <div class="main-grid">
        <!-- Live Camera Feed -->
        <div class="video-container">
            <img id="videoFeed" src="/video_feed" alt="Camera Feed"
                 onerror="this.style.display='none'; document.getElementById('videoOffline').style.display='flex';">
            <div id="videoOffline" class="video-offline" style="display:none;">
                <svg width="32" height="32" fill="#555" viewBox="0 0 24 24"><path d="M17 10.5V7c0-.55-.45-1-1-1H4c-.55 0-1 .45-1 1v10c0 .55.45 1 1 1h12c.55 0 1-.45 1-1v-3.5l4 4v-11l-4 4z"/></svg>
                Camera Offline
            </div>
        </div>

        <!-- Control Panel (Right Side) -->
        <div class="control-panel">

            <!-- START / STOP / PAUSE -->
            <div class="card">
                <div class="card-title">Motor Control</div>
                <div class="action-buttons">
                    <button class="btn btn-start" id="btnStart" onclick="sendCommand('start')">
                        ▶ START
                    </button>
                    <button class="btn btn-stop" id="btnStop" onclick="sendCommand('stop')" disabled>
                        ■ STOP
                    </button>
                    <button class="btn btn-pause" id="btnPause" onclick="sendCommand('pause')" disabled>
                        ⏸ PAUSE
                    </button>
                </div>
            </div>

            <!-- COLOR SELECTION -->
            <div class="card">
                <div class="card-title">Target Line Color</div>
                <div class="color-grid">
                    <button class="color-btn color-yellow active" id="btnYellow"
                            onclick="setColor('yellow')">🟡 Yellow</button>
                    <button class="color-btn color-green" id="btnGreen"
                            onclick="setColor('green')">🟢 Green</button>
                    <button class="color-btn color-blue" id="btnBlue"
                            onclick="setColor('blue')">🔵 Blue</button>
                </div>
            </div>

            <!-- SPEED CONTROL -->
            <div class="card">
                <div class="card-title">Base Speed</div>
                <div class="speed-row">
                    <input type="range" id="speedSlider" min="10" max="80" value="35"
                           oninput="setSpeed(this.value)">
                    <div class="speed-value" id="speedValue">35%</div>
                </div>
            </div>

            <!-- MOTOR OUTPUT BARS -->
            <div class="card">
                <div class="card-title">Motor Output</div>
                <div class="motor-bars">
                    <div class="motor-bar-group">
                        <div class="motor-bar-label">Left</div>
                        <div class="motor-bar-track">
                            <div class="motor-bar-fill" id="leftBar" style="height:0%"></div>
                        </div>
                        <div class="motor-bar-value" id="leftValue">0%</div>
                    </div>
                    <div class="motor-bar-group">
                        <div class="motor-bar-label">Right</div>
                        <div class="motor-bar-track">
                            <div class="motor-bar-fill" id="rightBar" style="height:0%"></div>
                        </div>
                        <div class="motor-bar-value" id="rightValue">0%</div>
                    </div>
                </div>
            </div>

            <!-- TELEMETRY -->
            <div class="card">
                <div class="card-title">Telemetry</div>
                <div class="telemetry-grid">
                    <div class="telem-item">
                        <div class="telem-label">Detection</div>
                        <div class="telem-value" id="telemDetect">—</div>
                    </div>
                    <div class="telem-item">
                        <div class="telem-label">FPS</div>
                        <div class="telem-value" id="telemFps">—</div>
                    </div>
                    <div class="telem-item">
                        <div class="telem-label">Steering</div>
                        <div class="telem-value" id="telemSteer">—</div>
                    </div>
                    <div class="telem-item">
                        <div class="telem-label">Error</div>
                        <div class="telem-value" id="telemError">—</div>
                    </div>
                </div>
            </div>

            <!-- Connection Info -->
            <div class="card" style="padding:12px 18px;">
                <span class="conn-dot conn-online" id="connDot"></span>
                <span style="font-size:0.75rem; color:#94a3b8;" id="connText">
                    Motors: <span id="motorMode">—</span>
                </span>
            </div>
        </div>
    </div>

    <script>
        // Polling interval for telemetry (ms)
        const POLL_INTERVAL = 200;

        function sendCommand(cmd) {
            fetch('/api/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({command: cmd})
            }).then(r => r.json()).then(updateUI);
        }

        function setColor(color) {
            fetch('/api/color', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({color: color})
            }).then(r => r.json()).then(updateUI);
        }

        function setSpeed(val) {
            document.getElementById('speedValue').textContent = val + '%';
            fetch('/api/speed', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({speed: parseFloat(val)})
            });
        }

        function updateUI(data) {
            if (!data) return;

            // Status Badge
            const badge = document.getElementById('statusBadge');
            if (!data.running) {
                badge.textContent = 'STOPPED';
                badge.className = 'status-badge status-stopped';
            } else if (data.paused) {
                badge.textContent = 'PAUSED';
                badge.className = 'status-badge status-paused';
            } else {
                badge.textContent = 'RUNNING';
                badge.className = 'status-badge status-running';
            }

            // Buttons
            document.getElementById('btnStart').disabled = data.running;
            document.getElementById('btnStop').disabled = !data.running;
            document.getElementById('btnPause').disabled = !data.running;
            document.getElementById('btnPause').textContent =
                data.paused ? '▶ RESUME' : '⏸ PAUSE';

            // Color buttons
            document.querySelectorAll('.color-btn').forEach(b => b.classList.remove('active'));
            if (data.active_color === 'yellow') document.getElementById('btnYellow').classList.add('active');
            if (data.active_color === 'green')  document.getElementById('btnGreen').classList.add('active');
            if (data.active_color === 'blue')   document.getElementById('btnBlue').classList.add('active');

            // Speed slider sync
            document.getElementById('speedSlider').value = data.base_speed;
            document.getElementById('speedValue').textContent = Math.round(data.base_speed) + '%';

            // Motor bars
            const lPct = Math.min(100, Math.abs(data.left_speed));
            const rPct = Math.min(100, Math.abs(data.right_speed));
            document.getElementById('leftBar').style.height = lPct + '%';
            document.getElementById('rightBar').style.height = rPct + '%';
            document.getElementById('leftValue').textContent = data.left_speed + '%';
            document.getElementById('rightValue').textContent = data.right_speed + '%';

            // Telemetry
            const detectEl = document.getElementById('telemDetect');
            if (data.detected) {
                detectEl.textContent = 'LOCKED';
                detectEl.className = 'telem-value tracking';
            } else {
                detectEl.textContent = 'LOST';
                detectEl.className = 'telem-value lost';
            }
            document.getElementById('telemFps').textContent = data.fps;
            document.getElementById('telemSteer').textContent = data.steering;
            document.getElementById('telemError').textContent = data.error;

            // Motor mode
            document.getElementById('motorMode').textContent = data.motor_mode;
        }

        // Periodic telemetry polling
        function pollStatus() {
            fetch('/api/status').then(r => r.json()).then(updateUI).catch(() => {
                document.getElementById('connDot').className = 'conn-dot conn-offline';
            });
        }
        setInterval(pollStatus, POLL_INTERVAL);
        pollStatus();

        // Keyboard shortcuts
        document.addEventListener('keydown', function(e) {
            const key = e.key.toLowerCase();
            if (key === 'y') setColor('yellow');
            else if (key === 'g') setColor('green');
            else if (key === 'b') setColor('blue');
            else if (key === ' ') { e.preventDefault(); sendCommand('pause'); }
            else if (key === 'enter') sendCommand('start');
            else if (key === 'escape') sendCommand('stop');
        });
    </script>
</body>
</html>
"""

robot_controller = None

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route('/video_feed')
def video_feed():
    """MJPEG live camera stream endpoint."""
    def generate():
        while True:
            with frame_lock:
                frame_data = latest_frame
            if frame_data is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
            else:
                # Send a small black placeholder frame
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting for camera...", (150, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 100, 100), 2)
                _, jpeg = cv2.imencode('.jpg', blank)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.033)  # ~30 FPS cap

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/status')
def api_status():
    return jsonify(state.to_dict())


@app.route('/api/command', methods=['POST'])
def api_command():
    data = request.get_json()
    cmd = data.get('command', '')

    with state.lock:
        if cmd == 'start':
            state.running = True
            state.paused = False
            print("[WEB] Robot STARTED")
        elif cmd == 'stop':
            state.running = False
            state.paused = False
            print("[WEB] Robot STOPPED")
        elif cmd == 'pause':
            if state.running:
                state.paused = not state.paused
                print(f"[WEB] Robot {'PAUSED' if state.paused else 'RESUMED'}")

    return jsonify(state.to_dict())


@app.route('/api/color', methods=['POST'])
def api_color():
    data = request.get_json()
    color = data.get('color', 'yellow').lower().strip()
    available = ["yellow", "green", "blue", "red", "black"]
    if color in available:
        with state.lock:
            state.active_color = color
        print(f"[WEB] Color changed to: {color.upper()}")
    return jsonify(state.to_dict())


@app.route('/api/speed', methods=['POST'])
def api_speed():
    data = request.get_json()
    speed = float(data.get('speed', config.BASE_SPEED))
    speed = max(10.0, min(80.0, speed))
    with state.lock:
        state.base_speed = speed
    return jsonify(state.to_dict())


def main():
    global robot_controller

    parser = argparse.ArgumentParser(description="Line Follower Web UI Server")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (0.0.0.0 for all). Default: 0.0.0.0")
    parser.add_argument("--port", type=int, default=5000, help="Web server port. Default: 5000")
    parser.add_argument("--cam", type=int, default=config.CAMERA_INDEX, help="Camera device index")
    args = parser.parse_args()

    print("=" * 56)
    print("   LINE FOLLOWER - WEB UI SERVER                     ")
    print("=" * 56)
    print(f"   Camera Index : {args.cam}")
    print(f"   Web UI URL   : http://0.0.0.0:{args.port}")
    print(f"   Access from  : http://<this-ip>:{args.port}")
    print("=" * 56)

    # Start robot control thread
    robot_controller = RobotController(camera_index=args.cam)
    robot_controller.start()

    # Give camera a moment to open
    time.sleep(1.0)

    # Start Flask web server
    try:
        app.run(host=args.host, port=args.port, threaded=True, debug=False)
    except KeyboardInterrupt:
        pass
    finally:
        if robot_controller:
            robot_controller.stop()
            robot_controller.join(timeout=3)
        print("[WEB] Server shut down.")


if __name__ == "__main__":
    main()
