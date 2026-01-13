# HAND_EYE_CALIB KNOWLEDGE BASE

**Type:** C++ Calibration Node
**Math:** AX=XB Solver

## OVERVIEW
Computes rigid body transform (Rotation/Translation) between Robot End-Effector and Camera Optical Frame. Solves AX=XB using captured pose pairs.

## STRUCTURE
```
hand_eye_calib/
├── src/               # C++ Solver (`hand_eye_calib.cpp`)
├── include/           # Headers
├── srv/               # Service definitions
└── scripts/           # Python orchestration
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| **Solver Core** | `src/hand_eye_calib.cpp` | Matrix math implementation |
| **Service** | `src/hand_eye_calib_srv.cpp` | ROS service callbacks |
| **Data Types** | `srv/HandEyeCalibData.srv` | Request/Response struct |
| **Test Script** | `scripts/hand_eye_bringup.py` | Auto-calibration seq |

## WORKFLOW
1. **Collect**: Robot Pose (PLC) + Camera Target (Vision).
2. **Solve**: Call `hand_eye_calib_srv` with N pairs.
3. **Result**: Publish TF / Save YAML.
