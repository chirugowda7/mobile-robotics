"""
Quick Camera Diagnostic Tool
----------------------------
Run: python test_camera.py

Scans all camera indices (0 to 4) using both DirectShow and Media Foundation
to find your external USB webcam and displays a test window.
"""

import cv2
import os
import time

print("=" * 60)
print("Scanning for connected webcams (indices 0 to 4)...")
print("=" * 60)

found_cameras = []

for idx in range(5):
    print(f"\nTesting Camera Index {idx}...")
    
    # Try DirectShow first with MJPG
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not cap.isOpened():
        cap.release()
        # Fallback to default backend
        cap = cv2.VideoCapture(idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if cap.isOpened():
        ret = False
        frame = None
        for _ in range(6):  # allow USB sensor warmup
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                break
            time.sleep(0.04)

        if ret and frame is not None and frame.size > 0:
            h, w = frame.shape[:2]
            print(f"  --> SUCCESS: Camera {idx} is WORKING! Frame size: {w}x{h}")
            found_cameras.append((idx, w, h))
        else:
            print(f"  --> Camera {idx} opened, but delivered 0 frames (likely IR/Hello sensor).")
        cap.release()
    else:
        print(f"  --> Camera {idx} not available.")

print("\n" + "=" * 60)
if not found_cameras:
    print("NO WORKING CAMERAS FOUND!")
    print("Check:")
    print("  1. Is the USB webcam firmly plugged in?")
    print("  2. In Windows Settings -> Privacy -> Camera, is 'Allow apps to access your camera' ON?")
    print("  3. Is another app (Zoom, Teams, Chrome, Camera App) using the webcam?")
else:
    print(f"Found {len(found_cameras)} working camera(s):")
    for idx, w, h in found_cameras:
        desc = "External USB Webcam" if idx > 0 else "Laptop / Primary Webcam"
        print(f"  - Index {idx}: {desc} ({w}x{h})")

    # Open live preview for the first external camera (or first available)
    preview_idx = found_cameras[0][0]
    for idx, _, _ in found_cameras:
        if idx > 0:  # prefer external
            preview_idx = idx
            break

    print(f"\nOpening live test preview for Camera {preview_idx}...")
    print("Press 'q' in the camera window to close.")

    cap = cv2.VideoCapture(preview_idx, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        cv2.putText(frame, f"Testing Camera Index: {preview_idx} (Press 'q' to exit)",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(f"Camera Test (Index {preview_idx})", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

print("\nDone.")
