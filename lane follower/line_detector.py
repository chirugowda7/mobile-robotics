"""
Line Detector using HSV Color Filtering, Morphological Clean-up, and Centroid Tracking.
Supports runtime switching between colors: Green, Blue, Yellow, Red, Black, etc.
"""

import cv2
import numpy as np
import config


class LineDetector:
    """
    Detects colored lines in video frames and calculates steering deviation error.
    """

    def __init__(self, initial_color="yellow"):
        self.color_ranges = config.load_color_ranges()
        self.active_color = initial_color.lower()
        if self.active_color not in self.color_ranges:
            print(f"[VISION] Warning: '{initial_color}' not found, defaulting to 'yellow'")
            self.active_color = "yellow"

        # Morphological structuring element
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

        # Smoothed error tracking
        self.last_valid_error = 0.0
        self.frames_without_detection = 0

    def set_color(self, color_name: str) -> bool:
        """Changes the active line tracking color."""
        color_name = color_name.lower().strip()
        if color_name in self.color_ranges:
            self.active_color = color_name
            print(f"[VISION] Active color changed to: {self.active_color.upper()}")
            return True
        else:
            print(f"[VISION] Color '{color_name}' not available. Options: {list(self.color_ranges.keys())}")
            return False

    def get_active_color(self) -> str:
        return self.active_color

    def get_available_colors(self) -> list:
        return list(self.color_ranges.keys())

    def cycle_next_color(self) -> str:
        """Cycles to the next available color in the list."""
        colors = self.get_available_colors()
        curr_idx = colors.index(self.active_color) if self.active_color in colors else -1
        next_color = colors[(curr_idx + 1) % len(colors)]
        self.set_color(next_color)
        return next_color

    def process_frame(self, frame):
        """
        Processes a single camera frame to detect the line.
        :param frame: BGR input image from camera
        :return: dict with keys:
            - 'detected': bool
            - 'error': float (-1.0 to 1.0)
            - 'cx': int (centroid x in full frame)
            - 'cy': int (centroid y in full frame)
            - 'mask': binary mask of ROI
            - 'annotated': BGR visual frame with debug overlays
            - 'active_color': str
        """
        h, w = frame.shape[:2]

        # Calculate Region of Interest (ROI) - Ground plane
        roi_y_start = int(h * config.ROI_START_RATIO)
        roi_y_end = int(h * config.ROI_END_RATIO)
        roi = frame[roi_y_start:roi_y_end, 0:w]
        roi_h, roi_w = roi.shape[:2]

        # Blur to reduce camera sensor noise
        blurred = cv2.GaussianBlur(roi, (7, 7), 0)

        # Convert to HSV color space
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Generate binary mask based on active color range
        color_info = self.color_ranges[self.active_color]
        lower = np.array(color_info["lower"], dtype=np.uint8)
        upper = np.array(color_info["upper"], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        # If color has a secondary range (e.g. Red wrapping across 180 Hue)
        if "lower2" in color_info and "upper2" in color_info:
            lower2 = np.array(color_info["lower2"], dtype=np.uint8)
            upper2 = np.array(color_info["upper2"], dtype=np.uint8)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask, mask2)

        # Clean up mask using morphological operations
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        annotated = frame.copy()
        # Draw ROI guide rectangle
        cv2.rectangle(annotated, (0, roi_y_start), (w - 1, roi_y_end), (255, 200, 0), 2)
        cv2.putText(annotated, f"ROI ({config.ROI_START_RATIO*100:.0f}%-{config.ROI_END_RATIO*100:.0f}%)",
                    (10, roi_y_start - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)

        # Center line marker
        frame_center_x = w // 2
        cv2.line(annotated, (frame_center_x, roi_y_start), (frame_center_x, roi_y_end), (200, 200, 200), 1, cv2.LINE_AA)

        detected = False
        error = 0.0
        centroid_x = frame_center_x
        centroid_y = (roi_y_start + roi_y_end) // 2

        if contours:
            # Select the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area >= config.MIN_CONTOUR_AREA:
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    # Centroid in ROI coordinates
                    cx_roi = int(M["m10"] / M["m00"])
                    cy_roi = int(M["m01"] / M["m00"])

                    # Convert to full frame coordinates
                    centroid_x = cx_roi
                    centroid_y = cy_roi + roi_y_start

                    # Normalized error: -1.0 (far left) to +1.0 (far right)
                    error = (centroid_x - frame_center_x) / float(frame_center_x)
                    detected = True
                    self.last_valid_error = error
                    self.frames_without_detection = 0

                    # Draw contour translated to full frame
                    shifted_contour = largest_contour + np.array([0, roi_y_start])
                    cv2.drawContours(annotated, [shifted_contour], -1, (0, 255, 0), 2)

                    # Draw centroid target
                    disp_col = color_info.get("display_color", (0, 255, 255))
                    cv2.circle(annotated, (centroid_x, centroid_y), 8, disp_col, -1)
                    cv2.circle(annotated, (centroid_x, centroid_y), 12, (255, 255, 255), 2)

                    # Draw steering vector line from frame center to centroid
                    cv2.arrowedLine(annotated, (frame_center_x, centroid_y), (centroid_x, centroid_y),
                                    (0, 0, 255) if abs(error) > 0.2 else (0, 255, 0), 3, tipLength=0.2)

                    # Bounding box
                    x, y, bw, bh = cv2.boundingRect(largest_contour)
                    cv2.rectangle(annotated, (x, y + roi_y_start), (x + bw, y + bh + roi_y_start), (0, 165, 255), 1)

        if not detected:
            self.frames_without_detection += 1
            # If line is briefly lost, retain last error direction for recovery
            error = self.last_valid_error

        return {
            "detected": detected,
            "error": error,
            "cx": centroid_x,
            "cy": centroid_y,
            "mask": mask,
            "annotated": annotated,
            "active_color": self.active_color,
            "frames_lost": self.frames_without_detection
        }
