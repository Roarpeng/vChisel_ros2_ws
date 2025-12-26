# Fix cv_bridge Missing Dependency

## Problem
The norm_calc package fails to compile because cv_bridge is not declared as a dependency, even though it's used in the source code.

## Root Cause
- `cv_bridge/cv_bridge.h` is included and used in `norm_calc_server.cpp`
- cv_bridge is not declared in `package.xml`
- `find_package(cv_bridge REQUIRED)` is missing from `CMakeLists.txt`
- cv_bridge is not in `ament_target_dependencies` for norm_calc_server

## Solution

### 1. Add cv_bridge to package.xml
**File**: `/home/bosch/vChisel_ros2_ws/src/norm_calc/package.xml`
- Add `<depend>cv_bridge</depend>` after the existing `<depend>` declarations (after opencv4)

### 2. Add cv_bridge to CMakeLists.txt
**File**: `/home/bosch/vChisel_ros2_ws/src/norm_calc/CMakeLists.txt`
- Add `find_package(cv_bridge REQUIRED)` in the find_package section (around line 23)
- Add `cv_bridge` to `ament_target_dependencies(norm_calc_server ...)` (around line 56)

## Verification Steps
After applying changes:
1. Clean build: `rm -rf build/ install/ log/`
2. Rebuild: `colcon build --packages-select norm_calc`
3. Source: `source install/setup.bash`

## Note
If cv_bridge is not installed on the system, install it first:
```bash
sudo apt install ros-humble-cv-bridge
```
