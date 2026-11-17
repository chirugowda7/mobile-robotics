# Orbbec Gemini E Depth Camera - Feature GUI Visualizer

A C++ GUI application using OpenCV to capture, visualize, analyze, and probe all streams and features of the **Orbbec Gemini E 3D Depth Camera** in real-time.

---

## 📌 Detected Camera Hardware & Port Connection

From hardware detection on this system (`lsusb` & V4L2 probe):
- **Camera Model**: Orbbec Gemini E 3D Depth Camera
- **RGB Sensor**: Bus 001 Device 008 (`ID 2bc5:055c`), mapped to `/dev/video2`
- **Depth Sensor**: Bus 001 Device 009 (`ID 2bc5:065c`)
- **Connection**: USB Bus 001 (High-Speed USB Hub)

---

## 🚀 Quick Start Guide

### 1. Build the C++ Application
```bash
cd /home/robotics/depth_camera
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 2. Run the C++ GUI Frame
```bash
./orbbec_depth_gui
```

### 3. Setup USB udev Permissions (If Depth Endpoint Access Fails)
```bash
cd /home/robotics/depth_camera
./scripts/setup_udev.sh
```

---

## 🎨 Visual Features & Multi-View Frame Layout

The application renders a 2x2 multi-view composite window displaying:

1. **Top-Left (Quad 1): Live RGB Stream**
   - Reticle targeting crosshair.
   - Real-time FPS counter, camera device info, timestamp.

2. **Top-Right (Quad 2): Colorized Depth Map Heatmap**
   - 16-bit millimeter depth rendering.
   - Dynamic depth clipping range sliders.
   - Interactive colormaps (`JET`, `TURBO`, `PLASMA`, `BONE`, `HOT`).
   - Target distance at reticle center in meters & millimeters.

3. **Bottom-Left (Quad 3): Visual Feature Keypoint Extractor**
   - Live ORB feature detection & landmark keypoints.
   - Canny edge detection mode.

4. **Bottom-Right (Quad 4): Depth Analytics & Interactive Probe**
   - Real-time depth distribution bar histogram (0.3m to 4.5m+ range).
   - Interactive mouse distance probe: Hover over any point in the frame to inspect precise `(X, Y)` coordinate distance.

---

## ⌨️ Keyboard Shortcuts

| Key | Action |
|:---|:---|
| `c` / `C` | Cycle colormap (`JET` ➔ `TURBO` ➔ `PLASMA` ➔ `BONE` ➔ `HOT`) |
| `f` / `F` | Toggle Feature Mode (`ORB Keypoints` ➔ `Canny Edges` ➔ `Off`) |
| `s` / `S` | Save snapshot frame to file (`orbbec_snapshot_<timestamp>.png`) |
| `h` / `H` | Toggle On-Screen Help Overlay |
| `q` / `ESC`| Quit Application |
