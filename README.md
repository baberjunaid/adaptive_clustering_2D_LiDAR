# Adaptive Clustering for 2D LiDAR in ROS

## Overview

This ROS package implements adaptive clustering for 2D LiDAR data. The adaptive clustering algorithm groups nearby points in the LiDAR scan based on adaptive thresholds, providing a more accurate representation of objects in the environment.

## Demo

![Demo GIF](demo.gif)

### Topics
- ROS
- LiDAR
- Adaptive Clustering
- Robotics
- Sensor Data
- Visualization
- Python
- Catkin
- Marker Array

### Tags
ROS, LiDAR, Adaptive Clustering, Robotics, Sensor Data, Visualization, Python, Catkin, Marker Array

## Prerequisites

- ROS (Robot Operating System)
- Catkin workspace

## Installation

1. Clone the repository to your catkin workspace:

    ```bash
    git clone https://github.com/your-username/adaptive_clustering_2D.git
    ```

2. Build the catkin workspace:

    ```bash
    cd path/to/your/catkin_workspace
    catkin_make
    source devel/setup.bash
    ```

## Usage

### Launching the Adaptive Clustering Node

To launch the adaptive clustering node with default settings (subscribing to the `/scan` topic):

```bash
roslaunch adaptive_clustering_2D adaptive_clustering.launch
```
To launch the adaptive clustering node with a custom scan topic (e.g., `/my_custom_scan`):

```bash
roslaunch adaptive_clustering_2D adaptive_clustering.launch scan_topic:=/my_custom_scan
```

## Parameters

- **`scan_topic` (default: `/scan`):** The LiDAR scan topic to subscribe to.
- **`cluster_size_min` (default: `3`):** Minimum size of a cluster to be considered.
- **`tolerance_factor` (default: `0.1`):** Factor used to calculate adaptive threshold.

Adjust these parameters in the launch file as needed.

## Citations
```bash
@article{gomez2023efficient,
title={Efficient Detection and Tracking of Human Using 3D LiDAR Sensor},
author={G{'o}mez, J. and Aycard, O. and Baber, J.},
journal={Sensors},
volume={23},
number={10},
pages={4720},
year={2023},
}
```


