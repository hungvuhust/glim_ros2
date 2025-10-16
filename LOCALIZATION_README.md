# GLIM Localization ROS2 Node

## Overview

`glim_localization_rosnode` is a ROS2 node for robot localization using a pre-built global map. It uses LiDAR data to estimate the robot's pose within the map using scan-to-scan and scan-to-map matching with graph optimization.

## Features

- **Hybrid Localization**: Combines scan-to-scan and scan-to-map matching for robust localization
- **Adaptive Confidence**: Automatically adjusts matching strategy based on confidence scores
- **Graph Optimization**: Uses GTSAM Fixed-Lag Smoother for optimal pose estimation
- **Marginalization**: Efficiently handles long sequences by marginalizing old frames
- **Keyframe Management**: Maintains keyframes for loop closure detection
- **ROS2 Integration**: Publishes pose, path, and diagnostics on ROS2 topics

## Installation

The localization node is built as part of the `glim_ros` package. Make sure all dependencies are installed:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select glim glim_ros
source install/setup.bash
```

## Usage

### 1. Prepare Global Map

First, you need a global map in PCD format or as submaps. You can create this using the standard GLIM SLAM:

```bash
# Run GLIM SLAM to build a map
ros2 launch glim_ros glim_ros.launch.py

# After mapping, save the map
# The map will be saved to /tmp/dump/globalmap.pcd
```

### 2. Set Initial Pose

You can set the initial pose in two ways:

#### Option A: Auto Initial Pose (Identity)
```bash
ros2 launch glim_ros glim_localization.launch.py \
  global_map_path:=/path/to/your/map.pcd \
  auto_initial_pose:=true
```

#### Option B: Manual Initial Pose
```bash
# Launch localization
ros2 launch glim_ros glim_localization.launch.py \
  global_map_path:=/path/to/your/map.pcd \
  auto_initial_pose:=false

# Set initial pose via ROS2 topic (in another terminal)
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, \
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --once
```

Or use RViz2's "2D Pose Estimate" tool.

### 3. Run Localization

#### Basic Usage
```bash
ros2 launch glim_ros glim_localization.launch.py \
  global_map_path:=/path/to/your/map.pcd
```

#### With Debug Mode
```bash
ros2 launch glim_ros glim_localization.launch.py \
  global_map_path:=/path/to/your/map.pcd \
  debug:=true
```

#### With Custom Config
```bash
ros2 launch glim_ros glim_localization.launch.py \
  global_map_path:=/path/to/your/map.pcd \
  config_path:=/path/to/your/config
```

## Topics

### Subscribed Topics

- `/imu` (`sensor_msgs/msg/Imu`): IMU data (optional, for future integration)
- `/points` (`sensor_msgs/msg/PointCloud2`): LiDAR point cloud data
- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`): Initial pose estimate

### Published Topics

- `/localization/pose` (`geometry_msgs/msg/PoseStamped`): Current estimated pose
- `/localization/pose_cov` (`geometry_msgs/msg/PoseWithCovarianceStamped`): Pose with covariance
- `/localization/path` (`nav_msgs/msg/Path`): Trajectory path

## Parameters

### Launch Parameters

- `config_path` (string, default: `<glim_pkg>/config`): Path to configuration directory
- `global_map_path` (string, required): Path to global map file (.pcd) or directory
- `auto_initial_pose` (bool, default: `false`): Use identity as initial pose
- `debug` (bool, default: `false`): Enable debug mode

### Configuration Parameters

Edit `config/config_localization.json`:

```json
{
  "localization": {
    "so_name": "liblocalization_cpu.so",
    "num_threads": 4,
    
    "scan_to_map_threshold": 0.7,
    "scan_to_scan_threshold": 0.5,
    
    "max_correspondence_distance": 1.0,
    "min_correspondences": 100,
    
    "regularization_weight": 1.0,
    "smoother_lag": 5.0,
    
    "ivox_resolution": 0.5,
    "keyframe_distance_threshold": 2.0,
    "keyframe_angle_threshold": 0.5
  }
}
```

## Visualization with RViz2

```bash
rviz2
```

Add displays:
1. **TF**: To visualize coordinate frames
2. **PoseStamped**: Topic `/localization/pose`
3. **Path**: Topic `/localization/path`
4. **PointCloud2**: Topic `/points` for LiDAR visualization
5. **Map**: Load your global map for reference

Set the Fixed Frame to `map`.

## Architecture

```
┌─────────────┐
│ Point Cloud │
└──────┬──────┘
       │
       v
┌─────────────────┐
│  Preprocessor   │
└──────┬──────────┘
       │
       v
┌──────────────────────────────┐
│   Async Localization         │
│  ┌────────────────────────┐  │
│  │ Scan-to-Scan Matching  │  │
│  └───────────┬────────────┘  │
│              │               │
│  ┌───────────v────────────┐  │
│  │ Scan-to-Map Matching   │  │
│  └───────────┬────────────┘  │
│              │               │
│  ┌───────────v────────────┐  │
│  │  Decision Strategy     │  │
│  └───────────┬────────────┘  │
│              │               │
│  ┌───────────v────────────┐  │
│  │  Graph Optimization    │  │
│  │  (Fixed-Lag Smoother)  │  │
│  └───────────┬────────────┘  │
│              │               │
│  ┌───────────v────────────┐  │
│  │   Marginalization      │  │
│  └────────────────────────┘  │
└──────────────────────────────┘
       │
       v
┌──────────────┐
│ Pose Output  │
└──────────────┘
```

## Differences from GLIM SLAM Node

| Feature | GLIM SLAM (`glim_rosnode`) | GLIM Localization (`glim_localization_rosnode`) |
|---------|---------------------------|------------------------------------------------|
| Purpose | Build map + estimate pose | Estimate pose only |
| Map | Creates new map | Uses pre-built map |
| Modules | Odometry + SubMapping + GlobalMapping | Localization only |
| Initialization | Starts from origin | Requires initial pose |
| Output | Trajectory + Map + Submaps | Trajectory only |
| Matching | Frame-to-model (iVox) | Hybrid (scan-to-scan + scan-to-map) |
| Memory | Growing (builds map) | Fixed (uses existing map) |

## Troubleshooting

### Localization Not Initialized
**Problem**: Node receives point clouds but doesn't localize.
**Solution**: Set initial pose via `/initialpose` topic or enable `auto_initial_pose`.

### Low Confidence Scores
**Problem**: Localization confidence is consistently low.
**Solution**: 
- Check if initial pose is close to actual position
- Adjust `scan_to_map_threshold` and `scan_to_scan_threshold` in config
- Ensure map quality is good
- Verify point cloud data is correct

### High CPU Usage
**Problem**: CPU usage is too high.
**Solution**:
- Reduce `num_threads` in config
- Increase `ivox_resolution` for faster processing
- Reduce point cloud density

### Drift Over Time
**Problem**: Pose drifts away from actual position.
**Solution**:
- Lower `smoother_lag` to keep optimization window smaller
- Increase `keyframe_distance_threshold` for more keyframes
- Enable loop closure (future feature)

## Example Workflow

### 1. Build a Map with GLIM SLAM
```bash
# Terminal 1: Run GLIM SLAM
ros2 launch glim_ros glim_ros.launch.py

# Terminal 2: Play your bag file
ros2 bag play your_mapping_session.bag

# Wait for mapping to complete, then save
# Map saved to /tmp/dump/globalmap.pcd
```

### 2. Use the Map for Localization
```bash
# Terminal 1: Run localization
ros2 launch glim_ros glim_localization.launch.py \
  global_map_path:=/tmp/dump/globalmap.pcd

# Terminal 2: Set initial pose (if not using auto mode)
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, \
    pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
                  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --once

# Terminal 3: Play your localization test bag
ros2 bag play your_test_session.bag

# Terminal 4: Visualize in RViz2
rviz2
```

## Performance

Typical performance on Intel i7 CPU:
- **Processing Time**: 10-50 ms per frame
- **Localization Frequency**: 10-20 Hz
- **Memory Usage**: 500 MB - 2 GB (depends on map size)
- **CPU Usage**: 20-40% (4 threads)

## Future Enhancements

- [ ] IMU integration for better velocity prediction
- [ ] Loop closure detection for global consistency
- [ ] Multi-resolution map support
- [ ] Online map update capability
- [ ] GPU acceleration support
- [ ] Detailed diagnostics publishing

## See Also

- [GLIM SLAM Documentation](../README.md)
- [Async Localization Module](../glim/src/glim/localization/ASYNC_LOCALIZATION_README.md)
- [Configuration Guide](../glim/config/README.md)

