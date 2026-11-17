#!/usr/bin/env python3
"""
Autonomous Camera True Line Follower with Cytron Motor Driver.
Features dynamic color selection (Green, Blue, Yellow), PID steering,
and real-time visual feedback overlay.
"""

import sys
import time
import argparse
import signal
import cv2
import numpy as np

import config
from motor_driver import CytronMotorDriver
from line_detector import LineDetector
from pid_controller import PIDController


class CameraLineFollower:
    def __init__(self, color="yellow", base_speed=config.BASE_SPEED, camera_index=config.CAMERA_INDEX, headless=False):
        self.headless = headless
        self.base_speed = float(base_speed)
        self.is_paused = False
        self.running = True

        # Initialize subsystems
        print("[SYSTEM] Initializing Line Follower Robot...")
        self.detector = LineDetector(initial_color=color)
        self.pid = PIDController()
        self.motors = CytronMotorDriver()

        # Initialize Camera
        print(f"[SYSTEM] Opening Camera device: {camera_index}...")
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            print(f"[ERROR] Could not open camera {camera_index}. Trying fallback indices 0 and 1...")
            for alt_idx in [0, 1, 2]:
                if alt_idx != camera_index:
                    self.cap = cv2.VideoCapture(alt_idx)
                    if self.cap.isOpened():
                        print(f"[SYSTEM] Camera opened on fallback index {alt_idx}")
                        break

        if not self.cap.isOpened():
            print("[FATAL] No video capture device could be opened. Exiting.")
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

        # Performance metrics
        self.fps_timer = time.time()
        self.fps = 0.0
        self.frame_count = 0
        self.last_detection_time = time.time()

        # Setup OS signal handlers for graceful shutdown (e.g., Ctrl+C)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, sig, frame):
        print("\n[SYSTEM] Termination signal received. Stopping robot...")
        self.running = False

    def draw_hud(self, frame, detection_result, left_spd, right_spd, steering):
        """Draws a professional heads-up telemetry display overlay."""
        h, w = frame.shape[:2]

        # Top dark status banner
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 55), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

        # 1. Color Selection Pills
        available_colors = ["yellow", "green", "blue"]
        x_offset = 15
        cv2.putText(frame, "TARGET:", (x_offset, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        x_offset += 65

        for col_name in available_colors:
            is_active = (col_name == self.detector.get_active_color())
            col_bgr = config.DEFAULT_COLOR_RANGES[col_name]["display_color"]

            box_w = 75
            box_h = 24
            y_box = 8

            if is_active:
                cv2.rectangle(frame, (x_offset, y_box), (x_offset + box_w, y_box + box_h), col_bgr, -1)
                text_col = (0, 0, 0) if col_name == "yellow" else (255, 255, 255)
            else:
                cv2.rectangle(frame, (x_offset, y_box), (x_offset + box_w, y_box + box_h), (60, 60, 60), 1)
                text_col = (160, 160, 160)

            # Draw hotkey hint
            label = f"[{col_name[0].upper()}]{col_name[1:]}"
            cv2.putText(frame, label, (x_offset + 6, y_box + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.42, text_col, 1)
            x_offset += box_w + 8

        # 2. Tracking Status Badge
        detected = detection_result["detected"]
        if self.is_paused:
            status_text = "PAUSED (SPACE)"
            status_col = (0, 165, 255)
        elif detected:
            status_text = "TRACKING"
            status_col = (0, 255, 0)
        else:
            lost_sec = time.time() - self.last_detection_time
            if lost_sec < config.LOST_LINE_TIMEOUT:
                status_text = f"SEARCHING ({lost_sec:.1f}s)"
                status_col = (0, 215, 255)
            else:
                status_text = "LOST - STOPPED"
                status_col = (0, 0, 255)

        cv2.putText(frame, status_text, (x_offset + 10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_col, 2)

        # 3. FPS Counter (Top Right)
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(frame, fps_text, (w - 110, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1)

        # 4. Telemetry info line
        telemetry = f"BaseSpd: {self.base_speed:.0f}% | Err: {detection_result['error']:+.2f} | Steer: {steering:+.1f} | L: {left_spd:.0f}% R: {right_spd:.0f}%"
        cv2.putText(frame, telemetry, (15, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1)

        # 5. Picture-in-Picture (PiP) Binary Mask Thumbnail
        mask = detection_result.get("mask")
        if mask is not None:
            pip_w = 120
            pip_h = 75
            pip_x = w - pip_w - 10
            pip_y = 65
            resized_mask = cv2.resize(mask, (pip_w, pip_h), interpolation=cv2.INTER_NEAREST)
            pip_bgr = cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(frame, (pip_x - 1, pip_y - 1), (pip_x + pip_w + 1, pip_y + pip_h + 1), (255, 255, 255), 1)
            frame[pip_y:pip_y + pip_h, pip_x:pip_x + pip_w] = pip_bgr
            cv2.putText(frame, "HSV MASK", (pip_x + 5, pip_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

        # 6. Bottom Key Shortcuts Helper
        bottom_y = h - 10
        shortcuts = "Keys: [Y]=Yellow  [G]=Green  [B]=Blue  [C]=Cycle  [SPACE]=Pause  [+/-]=Speed  [Q]=Quit"
        cv2.putText(frame, shortcuts, (15, bottom_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)

    def run(self):
        """Main control loop."""
        print("\n=======================================================")
        print("  CAMERA TRUE LINE FOLLOWER ROBOT ACTIVE")
        print("=======================================================")
        print(f"  Target Color     : {self.detector.get_active_color().upper()}")
        print(f"  Base Speed       : {self.base_speed}%")
        print(f"  Controls: [Y] Yellow | [G] Green | [B] Blue | [SPACE] Pause | [Q] Quit")
        print("=======================================================\n")

        try:
            while self.running:
                loop_start = time.time()
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    print("[WARNING] Frame capture dropped, retrying...")
                    time.sleep(0.01)
                    continue

                # 1. Image Processing & Line Detection
                result = self.detector.process_frame(frame)
                detected = result["detected"]
                error = result["error"]

                # 2. Motor Steering Logic via PID
                if self.is_paused:
                    left_spd = 0.0
                    right_spd = 0.0
                    steering = 0.0
                    self.motors.stop()
                elif detected:
                    self.last_detection_time = time.time()
                    # Calculate PID steering correction
                    steering = self.pid.update(error)

                    # Compute speeds with active reverse-pivot turning
                    left_spd, right_spd = config.compute_wheel_speeds(self.base_speed, steering, error)
                    self.motors.set_speeds(left_spd, right_spd)
                else:
                    # Line lost handling
                    lost_duration = time.time() - self.last_detection_time
                    if lost_duration < config.LOST_LINE_TIMEOUT:
                        # Recovery behavior: actively spin in last known direction
                        pivot_speed = self.base_speed * 0.75
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
                        # Full safety stop when line is definitively gone
                        left_spd = 0.0
                        right_spd = 0.0
                        steering = 0.0
                        self.motors.stop()
                        self.pid.reset()

                # 3. FPS Calculation
                self.frame_count += 1
                if time.time() - self.fps_timer >= 1.0:
                    self.fps = self.frame_count / (time.time() - self.fps_timer)
                    self.frame_count = 0
                    self.fps_timer = time.time()

                # 4. Visualization & Keyboard Input
                if not self.headless:
                    display_frame = result["annotated"]
                    self.draw_hud(display_frame, result, left_spd, right_spd, steering)
                    cv2.imshow("Camera True Line Follower - Cytron Driver", display_frame)

                    key = cv2.waitKey(1) & 0xFF
                    if key in [ord('q'), ord('Q'), 27]:  # 'q' or ESC
                        print("[USER] Quit requested via keyboard.")
                        break
                    elif key in [ord('y'), ord('Y')]:
                        self.detector.set_color("yellow")
                        self.pid.reset()
                    elif key in [ord('g'), ord('G')]:
                        self.detector.set_color("green")
                        self.pid.reset()
                    elif key in [ord('b'), ord('B')]:
                        self.detector.set_color("blue")
                        self.pid.reset()
                    elif key in [ord('c'), ord('C')]:
                        next_c = self.detector.cycle_next_color()
                        self.pid.reset()
                    elif key == ord(' '):  # SPACE toggles pause
                        self.is_paused = not self.is_paused
                        if self.is_paused:
                            self.motors.stop()
                            print("[USER] Robot PAUSED.")
                        else:
                            print("[USER] Robot RESUMED.")
                    elif key in [ord('+'), ord('=')]:
                        self.base_speed = min(config.MAX_SPEED, self.base_speed + 5.0)
                        print(f"[USER] Base speed increased to: {self.base_speed}%")
                    elif key in [ord('-'), ord('_')]:
                        self.base_speed = max(10.0, self.base_speed - 5.0)
                        print(f"[USER] Base speed decreased to: {self.base_speed}%")
                else:
                    # In headless mode, log status periodically
                    if self.frame_count % 30 == 0:
                        status_str = "TRACKING" if detected else "LOST"
                        print(f"[{status_str}] Color: {self.detector.get_active_color()} | Err: {error:+.2f} | L: {left_spd:.1f}% R: {right_spd:.1f}% | FPS: {self.fps:.1f}")

        finally:
            self.shutdown()

    def shutdown(self):
        """Clean resource disposal."""
        print("\n[SYSTEM] Shutting down motors and camera...")
        try:
            self.motors.stop()
            self.motors.cleanup()
        except Exception as e:
            print(f"[ERROR] Motor shutdown error: {e}")

        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"[ERROR] Camera release error: {e}")
        print("[SYSTEM] Line follower terminated cleanly.")


def main():
    parser = argparse.ArgumentParser(description="Autonomous Camera True Line Follower with Cytron Motor Driver")
    parser.add_argument("--color", type=str, default="yellow", choices=["yellow", "green", "blue", "red", "black"],
                        help="Initial line color to follow (yellow, green, blue). Default: yellow")
    parser.add_argument("--speed", type=float, default=config.BASE_SPEED,
                        help=f"Base cruising speed (0 - 100). Default: {config.BASE_SPEED}")
    parser.add_argument("--cam", type=int, default=config.CAMERA_INDEX,
                        help=f"Camera device index (/dev/videoX). Default: {config.CAMERA_INDEX}")
    parser.add_argument("--headless", action="store_true",
                        help="Run without displaying OpenCV window (ideal for SSH sessions)")

    args = parser.parse_args()

    follower = CameraLineFollower(
        color=args.color,
        base_speed=args.speed,
        camera_index=args.cam,
        headless=args.headless
    )
    follower.run()


if __name__ == "__main__":
    main()
