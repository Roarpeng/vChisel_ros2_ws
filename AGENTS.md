# vChisel_ros2_ws KNOWLEDGE BASE

**Generated:** 2026-01-13
**Type:** ROS 2 (Humble) Workspace
**Domain:** Industrial Automation / Vision / PLC

## OVERVIEW
Industrial vision system interfacing Depth Cameras with Siemens PLCs. Orchestrates point cloud processing (`norm_calc`), PLC communication (`snap_7`), and hand-eye calibration (`hand_eye_calib`).

## STRUCTURE
```
vChisel_ros2_ws/
├── src/
│   ├── norm_calc/       # Point Cloud Processing (C++ PCL)
│   ├── snap_7/          # PLC Communication (Python Snap7)
│   ├── hand_eye_calib/  # Robot-Camera Calibration (C++)
│   └── vision_opencv/   # OpenCV/ROS Bridge (Metapackage)
├── scripts/             # Global utilities
└── launch/              # System-wide launch files
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| **Point Cloud Logic** | `src/norm_calc/src/norm_calc.cpp` | PCL filters, normal estimation |
| **PLC Comms** | `src/snap_7/snap_7/plc_client_node.py` | DB read/write, state machine |
| **Calibration** | `src/hand_eye_calib/src/` | AX=XB solver |
| **Config** | `src/norm_calc/config/` | ROI, filter params |
| **System Launch** | `./start_system.sh` | One-click startup |

## WORKFLOW
1. **Trigger**: PLC sets specific DB address (default: 110).
2. **Capture**: System receives point cloud.
3. **Process**: `norm_calc` filters, downsamples, calculates normals.
4. **Response**: Result written back to PLC DB.

## COMMANDS
```bash
# Build specific packages
colcon build --packages-select norm_calc snap_7

# Run System
./start_system.sh

# Manual Launch
ros2 launch snap_7 snap_7.launch.py
ros2 launch norm_calc norm_calc_launch.py
```

## CONVENTIONS
- **Hybrid Stack**: C++ for heavy math (PCL), Python for logic/comms (Snap7).
- **PLC Protocol**: Uses Snap7 (S7 protocol). Requires strictly defined DB layout.
- **Coordinates**: Follows ROS REP-103 (X-forward, Z-up).

## NOTES
- **Dependencies**: Requires `python-snap7`, `libpcl-dev`, `ros-humble-desktop`.
- **Network**: PLC and PC must be on same subnet. Check `plc_client_node.py` for IP config.
