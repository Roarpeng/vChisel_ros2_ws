# NORM_CALC KNOWLEDGE BASE

**Type:** C++ ROS 2 Node (PCL)
**Domain:** Point Cloud Processing (Concrete Chisel Optimization)

## OVERVIEW
Processes raw depth camera data to extract surface normals for concrete chiseling tasks. Optimized for "Static Eye-in-Hand" + "Vertical Concrete" scenarios.
Features robust noise filtering and edge detection to prevent tool slippage.

## STRUCTURE
```
norm_calc/
├── src/                # Implementation
│   ├── norm_calc.cpp   # Pipeline: PassThrough -> Voxel -> OMP Normal -> Edge Filter
│   └── chisel_box.cpp  # Scoring: Flatness >> Height, Random Mode with Safety Check
├── include/            # Headers
├── config/             # Runtime parameters (Radius=2.5cm, Strict Weights)
└── launch/             # ROS 2 launch files
```

## ALGORITHM PIPELINE
1. **PassThrough**: ROI clipping.
2. **VoxelGrid**: 3mm downsampling.
3. **OMP Normal**: Multi-threaded normal estimation (Radius=2.5cm).
4. **Edge Filter (New)**: Double-scale consistency check (2.5cm vs 5.0cm). Rejects edges (>20 deg diff).
5. **Chisel Scoring**:
   - **Verticality**: High weight (Angle < 15 deg).
   - **Flatness**: High weight (Low curvature).
   - **Height**: Zero weight (Ignore protrusions to avoid loose debris).

## KEY PARAMETERS (Optimized)
- `SEARCH_RADIUS`: **0.025** (2.5cm) - Matches tool size, filters debris.
- `ANGLE_WEIGHT`: **20.0** - Extreme penalty for non-vertical surfaces.
- `HEIGHT_WEIGHT`: **0.0** - Do not prioritize protrusions.
- `STRICT_NORM_TH`: **0.90** - Only chisels solid, flat rock.

## SAFETY
- **Random Mode**: Now validates *real* normals before selecting (no fake normals).
- **Edge Detection**: Prevents chiseling on corners/edges/cracks.
