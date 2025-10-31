# Warehouse Inventory Management Drone System (Non-GPS Autonomous Navigation)

An autonomous indoor drone platform for warehouse inventory scanning and SKU inspection without relying on GPS. The system is built to improve stock verification efficiency and reduce manual labor in large warehouse environments.

## System Overview

This project targets a custom drone equipped with multiple Intel RealSense cameras and a Jetson Orin NX onboard computer. It performs visual-inertial odometry, 3D mapping, and SKU inspection using ROS2 Humble and RTAB-Map.

Hardware & sensors
- Jetson Orin NX (onboard computer)
- 4 Intel RealSense cameras for Visual-Inertial Odometry (VIO) and 360° localization
- 1 downward-facing Intel RealSense for altitude estimation and floor reference

Core components
- RTAB-Map: real-time 3D mapping and loop closure
- ROS2 Humble: middleware for control, navigation, and communication
- Custom flight stack + perception pipeline

## Key Capabilities

- ✔️ Autonomous flight inside large warehouses (GPS-denied)
- ✔️ 3D map generation for aisle and slot navigation
- ✔️ Automatic scanning of storage racks
- ✔️ SKU presence verification (label / barcode based)
- ✔️ Potential integration with Warehouse Management Systems (WMS)

## Tech Stack

- Jetson Orin NX
- Intel RealSense (D455 / T265 or similar)
- ROS2 Humble
- RTAB-Map (rtabmap_ros)
- librealsense2 and realsense-ros
- Python (ROS2 nodes and scripts)

## Repository layout

- slam/
  - launch/ — ROS2 Python launch files (e.g. `slam.launch.py`, `realsense.launch.py`, `rtabmap.launch.py`, `qrdetection.launch.py`, etc.)
  - config/ — sensor and filter configuration (e.g. `imu_filter.yaml`)
  - params/ — RTAB-Map and other parameter files (e.g. `rtabmap.yaml`)
  - scripts/ — ROS2 nodes and helpers (height fusion, QR detection, image sync, etc.)
  - package.xml, setup.py — ROS2 package metadata and Python packaging
  - LICENSE — project license for the `slam` package
  - best.pt — (model weights, used by perception components)

## Getting started (high-level)

Prerequisites
- Ubuntu 22.04 (recommended for ROS2 Humble on Jetson)
- ROS2 Humble installed and sourced
- Colcon build tools
- RTAB-Map and `rtabmap_ros` installed
- Intel RealSense SDK (`librealsense2`) and `realsense-ros` driver
- Python dependencies (check `setup.py` and `requirements` in the `slam` package)

Setup example (on the Jetson Orin NX)

1. Install ROS2 Humble and development tools: follow ROS2 official instructions for Ubuntu 22.04.
2. Install RTAB-Map and `rtabmap_ros`:
   - sudo apt install ros-humble-rtabmap-ros (or build from source if needed)
3. Install Intel RealSense SDK and ROS wrapper (`librealsense2`, `ros-humble-realsense2-camera`) following official guides.
4. Build the workspace

```bash
# from the workspace root
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launching

Examples of launching core components (replace camera indices and configs as needed):

- Launch the RealSense camera stack (example):

```bash
ros2 launch slam realsense.launch.py
```

- Launch RTAB-Map with SLAM pipeline and VIO bridge:

```bash
ros2 launch slam slam.launch.py
# or
ros2 launch slam rtabmap.launch.py
```

- Launch QR/barcode detection and SKU inspection:

```bash
ros2 launch slam qrdetection.launch.py
```

See `slam/launch/` for additional launch files (triple/quad camera configurations, voxel filtering, and more).

## Development notes

- ROS2 nodes and utility scripts are under `slam/slam/scripts/` (e.g. `height_fusion_node.py`, `qrdetection_node.py`, `rgbd_images_sync.py`).
- Tweak RTAB-Map parameters in `slam/params/rtabmap.yaml` and IMU filtering in `slam/config/imu_filter.yaml`.
- Model weights (e.g. `best.pt`) used by perception components are included under `slam/` — confirm license and usage before distribution.

## Contributing

Contributions are welcome. Open an issue to discuss features or fixes before submitting a PR. Keep changes focused and provide tests where possible.

## License

See `slam/LICENSE` for license details of the `slam` package. Confirm any third-party components' licenses (RTAB-Map, librealsense, models) before redistribution.

## Contact

Project: Warehouse Inventory Management Drone System
Repository: Harshitpatro/WarehouseDrone

For questions, issues, or collaboration ideas, please open an issue in this repository.
