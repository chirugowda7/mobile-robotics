#!/usr/bin/env python3
"""
Interactive HSV Color Calibration Tool for Camera Line Following.
Allows live tuning of HSV thresholds for Yellow, Green, and Blue lines
with real-time binary mask feedback and saving to color_config.json.
"""

import sys
import argparse
import cv2
import numpy as np
import config


def nothing(x):
    pass


class ColorCalibrator:
    def __init__(self, camera_index=config.CAMERA_INDEX, target_color="yellow"):
        self.camera_index = camera_index
        self.target_color = target_color.lower()
        self.color_ranges = config.load_color_ranges()

        if self.target_color not in self.color_ranges:
            self.target_color = "yellow"

        self.window_name = "HSV Color Tuner (Press 'S' to Save, '1' Yellow, '2' Green, '3' Blue, 'Q' Quit)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1000, 700)

        self._create_trackbars()

        print(f"[CAM] Opening camera index {self.camera_index}...")
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            print(f"[ERROR] Could not open camera {self.camera_index}. Trying 0 and 1...")
            for idx in [0, 1]:
                self.cap = cv2.VideoCapture(idx)
                if self.cap.isOpened():
                    break

        if not self.cap.isOpened():
            print("[FATAL] Camera could not be opened.")
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)

    def _create_trackbars(self):
        curr_range = self.color_ranges[self.target_color]
        lower = curr_range["lower"]
        upper = curr_range["upper"]

        cv2.createTrackbar("H Min", self.window_name, lower[0], 179, nothing)
        cv2.createTrackbar("H Max", self.window_name, upper[0], 179, nothing)
        cv2.createTrackbar("S Min", self.window_name, lower[1], 255, nothing)
        cv2.createTrackbar("S Max", self.window_name, upper[1], 255, nothing)
        cv2.createTrackbar("V Min", self.window_name, lower[2], 255, nothing)
        cv2.createTrackbar("V Max", self.window_name, upper[2], 255, nothing)

    def _update_trackbars_for_color(self, color_name):
        self.target_color = color_name
        curr_range = self.color_ranges[self.target_color]
        lower = curr_range["lower"]
        upper = curr_range["upper"]

        cv2.setTrackbarPos("H Min", self.window_name, lower[0])
        cv2.setTrackbarPos("H Max", self.window_name, upper[0])
        cv2.setTrackbarPos("S Min", self.window_name, lower[1])
        cv2.setTrackbarPos("S Max", self.window_name, upper[1])
        cv2.setTrackbarPos("V Min", self.window_name, lower[2])
        cv2.setTrackbarPos("V Max", self.window_name, upper[2])
        print(f"[CALIB] Switched to tuning color: {self.target_color.upper()}")

    def run(self):
        print("\n=======================================================")
        print("          HSV COLOR CALIBRATION TOOL                   ")
        print("=======================================================")
        print(f" Tuning Color : {self.target_color.upper()}")
        print(" Controls:")
        print("   [1] Tune Yellow | [2] Tune Green | [3] Tune Blue")
        print("   [S] Save current thresholds to config")
        print("   [Q] / [ESC] Quit")
        print("=======================================================\n")

        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

        try:
            while True:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    continue

                h, w = frame.shape[:2]
                roi_y_start = int(h * config.ROI_START_RATIO)
                roi_y_end = int(h * config.ROI_END_RATIO)
                roi = frame[roi_y_start:roi_y_end, 0:w]

                # Get Trackbar positions
                h_min = cv2.getTrackbarPos("H Min", self.window_name)
                h_max = cv2.getTrackbarPos("H Max", self.window_name)
                s_min = cv2.getTrackbarPos("S Min", self.window_name)
                s_max = cv2.getTrackbarPos("S Max", self.window_name)
                v_min = cv2.getTrackbarPos("V Min", self.window_name)
                v_max = cv2.getTrackbarPos("V Max", self.window_name)

                # Update in-memory range
                self.color_ranges[self.target_color]["lower"] = [h_min, s_min, v_min]
                self.color_ranges[self.target_color]["upper"] = [h_max, s_max, v_max]

                # Process ROI
                blurred = cv2.GaussianBlur(roi, (7, 7), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
                upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)

                # Morphology
                mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
                mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel_close)

                # Contour centroid tracking preview
                contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                preview_roi = roi.copy()
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest)
                    if area >= config.MIN_CONTOUR_AREA:
                        M = cv2.moments(largest)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            cv2.drawContours(preview_roi, [largest], -1, (0, 255, 0), 2)
                            cv2.circle(preview_roi, (cx, cy), 7, (0, 0, 255), -1)

                # Create Composite View
                # Top: Full Camera frame with ROI outline
                vis_frame = frame.copy()
                cv2.rectangle(vis_frame, (0, roi_y_start), (w - 1, roi_y_end), (0, 255, 255), 2)
                cv2.putText(vis_frame, f"TUNING: {self.target_color.upper()} (H:[{h_min},{h_max}] S:[{s_min},{s_max}] V:[{v_min},{v_max}])",
                            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

                # Bottom: Mask (left) and Detected Line ROI (right)
                mask_bgr = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR)
                half_w = w // 2
                scaled_h = 200
                mask_display = cv2.resize(mask_bgr, (half_w, scaled_h))
                roi_display = cv2.resize(preview_roi, (half_w, scaled_h))
                cv2.putText(mask_display, "CLEANED MASK", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(roi_display, "TRACKED ROI", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                bottom_row = np.hstack([mask_display, roi_display])
                composite = np.vstack([vis_frame, bottom_row])

                cv2.imshow(self.window_name, composite)

                key = cv2.waitKey(20) & 0xFF
                if key in [ord('q'), ord('Q'), 27]:
                    break
                elif key in [ord('s'), ord('S')]:
                    config.save_color_ranges(self.color_ranges)
                    print(f"[SAVED] Saved thresholds for {self.target_color.upper()} to disk!")
                elif key == ord('1'):
                    self._update_trackbars_for_color("yellow")
                elif key == ord('2'):
                    self._update_trackbars_for_color("green")
                elif key == ord('3'):
                    self._update_trackbars_for_color("blue")

        finally:
            self.cap.release()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Calibrate HSV thresholds for camera line follower")
    parser.add_argument("--color", type=str, default="yellow", choices=["yellow", "green", "blue", "red", "black"],
                        help="Initial color to calibrate. Default: yellow")
    parser.add_argument("--cam", type=int, default=config.CAMERA_INDEX,
                        help="Camera device index. Default: 0")
    args = parser.parse_args()

    calibrator = ColorCalibrator(camera_index=args.cam, target_color=args.color)
    calibrator.run()


if __name__ == "__main__":
    main()
