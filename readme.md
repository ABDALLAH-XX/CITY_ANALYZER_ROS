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
