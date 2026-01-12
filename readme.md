# City Analyzer ROS 2

A high-performance ROS 2 package for semantic analysis and segmentation of 3D urban point clouds (Lidar/PLY).

## 🚀 Features
- **Preprocessing**: Voxel Grid filtering to optimize RAM usage and processing speed.
- **Ground Segmentation**: Road plane extraction using the RANSAC algorithm.
- **Object Extraction**: Isolation of non-ground elements (buildings, vehicles, street furniture).
- **Real-time Optimization**: Asynchronous timers and QoS (Best Effort) policies for smooth visualization in RViz2.

## Project Structure
```text
city_analyzer_ros/
├── data/                  # PLY files (e.g., Lille_1.ply)
├── include/               # Header files (.hpp)
├── src/                   # C++ Source code (PCL & ROS 2)
├── launch/                # Launch scripts for automation
└── rviz/                  # Pre-configured RViz display settings
'''text
## Installation & Build
1. Prerequisites

    OS: Ubuntu 22.04 (Humble) or 20.04 (Foxy/Galactic)

    Framework: ROS 2

    Library: PCL (Point Cloud Library

2. Build Instructions
    # Navigate to your workspace
    cd ~/ros2_ws

    # Build
    colcon build --system-install
    or Build with limited CPU workers to save RAM
    colcon build --symlink-install --parallel-workers 2

    # Source the setup file
    source install/setup.bash

3. Run
    To launch the full pipeline (Processing Node + RViz2 Visualization):
    ros2 launch city_analyzer_ros city_visualizer.launch.py

4. Configuration

    You can fine-tune the processing thresholds within the launch file or main.cpp:

        voxel_leaf_size: Resolution of the downsampling (default: 0.2m).

        distance_threshold: RANSAC tolerance for plane detection (default: 0.15m).

        cluster_tolerance: Distance between points to form a cluster (default: 0.5m).

Author

    Abdallah
