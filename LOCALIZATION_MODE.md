# GLIM ROS2 - Localization Mode

This document explains how to use the localization mode in glim_ros, which allows the system to localize against a pre-built map instead of building a new map.

## Overview

Localization mode enables the following features:
- Load and use pre-built maps for localization
- Continuous odometry estimation against the pre-built map
- Sub-mapping with localization constraints
- Manual relocalization via ROS service or `/initialpose` topic
- Automatic relocalization on initialization

## Prerequisites

1. A pre-built map created by running glim_ros in standard SLAM mode
2. The map should be saved using the `save()` method, typically in a directory structure like:
   ```
   /path/to/map/
   ├── graph.txt
   ├── 000000/
   ├── 000001/
   ├── 000002/
   └── ...
   ```

## Configuration

### 1. Create a Localization Configuration File

Copy and modify the example configuration:

```bash
cp src/glim_localization/config/velodyne/config_ros.json \
   src/glim_localization/config/velodyne/config_ros_localization.json
```

Or use the provided template at `config/velodyne/config_ros_localization.json`.

### 2. Key Configuration Parameters

In your `config_ros_localization.json`, set the following parameters:

```json
{
  "glim_ros": {
    "localization_mode": true,
    "map_path": "/path/to/your/prebuilt/map",
    "enable_local_mapping": true,
    "enable_global_mapping": true,

    // Update extension modules for localization
    "extension_modules": [
      "libmemory_monitor.so",
      "liblocalization_viewer.so",
      "librviz_viewer.so"
    ],

    // Other standard parameters...
    "imu_topic": "/imu",
    "points_topic": "/velodyne_points",
    ...
  }
}
```

**Important Parameters:**
- `localization_mode`: Set to `true` to enable localization mode
- `map_path`: Absolute or relative path to the pre-built map directory
  - If relative, it will be resolved relative to the glim package directory
- `enable_local_mapping`: Recommended to keep `true` for better accuracy
- `enable_global_mapping`: Must be `true` for localization to work

### 3. Localization Algorithm Configuration

Create or modify `config_global_mapping.json` to specify the localization module:

```json
{
  "global_mapping": {
    "so_name": "libglobal_mapping.so",
    "localization_so_name": "liblocalization.so",

    // Localization-specific parameters
    "max_localization_distance": 5.0,
    "min_localization_overlap": 0.15,
    "linear_search_window": 8.0,
    "angular_search_window": 0.15,
    "num_keep_submaps": 3,

    // Standard mapping parameters
    "enable_imu": true,
    "enable_optimization": true,
    ...
  }
}
```

**Localization Parameters Explained:**
- `max_localization_distance` (default: 5.0m): Maximum distance to search for matching pre-built submaps
- `min_localization_overlap` (default: 0.15): Minimum overlap score required for successful localization
- `linear_search_window` (default: 8.0m): Linear search range for pose candidates during relocalization
- `angular_search_window` (default: 0.15 rad ≈ 8.6°): Angular search range for pose candidates
- `num_keep_submaps` (default: 3): Number of active submaps to maintain in memory (to limit memory usage)

## Usage

### 1. Building a Map (Standard SLAM Mode)

First, create a map using standard SLAM mode:

```bash
ros2 launch glim_ros glim_ros.launch.py \
  config_path:=config/velodyne
```

When finished, save the map:
```bash
ros2 service call /glim_ros/save_map std_srvs/srv/Trigger
```

Or programmatically:
```cpp
glim_ros->wait(true);
glim_ros->save("/path/to/save/map");
```

### 2. Running in Localization Mode

Launch glim_ros with your localization configuration:

```bash
ros2 launch glim_ros glim_ros.launch.py \
  config_path:=config/velodyne \
  config_ros:=config_ros_localization.json
```

The system will:
1. Load the pre-built map specified in `map_path`
2. Start odometry estimation
3. Create new submaps that are matched against the pre-built map
4. Continuously localize within the pre-built map

### 3. Manual Relocalization

There are two ways to manually trigger relocalization:

#### Method 1: Using /initialpose Topic (RViz)

In RViz, use the "2D Pose Estimate" tool:

1. Click the "2D Pose Estimate" button in RViz
2. Click and drag on the map to set the initial pose
3. The system will automatically trigger relocalization

This publishes to `/initialpose` topic (`geometry_msgs/PoseWithCovarianceStamped`).

#### Method 2: Using ROS Service

Call the relocalization service:

```bash
ros2 service call /glim_ros/trigger_relocalization std_srvs/srv/Trigger
```

This uses the most recently set initial pose (from `/initialpose` or default identity).

**Note:** Set the initial pose via `/initialpose` before calling the service for best results.

### 4. Programmatic Usage

```cpp
// In your ROS2 node
auto client = node->create_client<std_srvs::srv::Trigger>(
    "/glim_ros/trigger_relocalization");

auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
auto result = client->async_send_request(request);

// Or publish initial pose
auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10);

geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
pose_msg.header.frame_id = "map";
pose_msg.header.stamp = node->now();
pose_msg.pose.pose.position.x = 10.0;
pose_msg.pose.pose.position.y = 20.0;
pose_msg.pose.pose.position.z = 0.0;
pose_msg.pose.pose.orientation.w = 1.0;

pose_pub->publish(pose_msg);
// Relocalization will be triggered automatically
```

## ROS Interfaces

### Topics

#### Subscribed Topics
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
  - Set initial pose for relocalization
  - Automatically triggers relocalization when received

#### Published Topics
(Same as standard SLAM mode)
- `/glim_ros/odom` - Odometry output
- Various `/glim_ros/markers` - Visualization markers
- TF transforms: `map -> odom -> base_link`

### Services

- `~/trigger_relocalization` (`std_srvs/srv/Trigger`)
  - Manually trigger relocalization using the current initial pose estimate
  - Returns success/failure status

## Typical Workflow

### Scenario 1: Robot Initialization at Known Location

1. Start the robot at a known location in the map
2. Launch glim_ros in localization mode
3. Use RViz "2D Pose Estimate" to set the approximate initial pose
4. The system will automatically relocalize and start tracking

### Scenario 2: Robot Lost or Kidnapped

1. Robot is operating normally in localization mode
2. Robot loses tracking (e.g., moved to unknown location)
3. User identifies the robot's location on the map
4. Use "2D Pose Estimate" in RViz to set the new pose
5. System automatically relocalizes

### Scenario 3: Periodic Relocalization

```bash
# In a script or launch file
while true; do
  sleep 300  # Every 5 minutes
  ros2 service call /glim_ros/trigger_relocalization std_srvs/srv/Trigger
done
```

## Monitoring and Debugging

### Check Localization Status

Monitor the logs for localization messages:

```bash
ros2 topic echo /rosout | grep -i "relocal\|localization"
```

You should see messages like:
- `Starting in localization mode`
- `Loading pre-built map from: ...`
- `Successfully loaded pre-built map`
- `Localization mode services initialized`
- `Received initial pose from /initialpose`
- `Auto-triggered relocalization`

### Visualization

Use the localization viewer extension module for visual feedback:

```json
"extension_modules": [
  "liblocalization_viewer.so",
  "librviz_viewer.so"
]
```

This will display:
- Pre-built map submaps (in gray or different color)
- Current active submaps (in color)
- Relocalization matches
- Pose graph

## Troubleshooting

### Issue: "Failed to load pre-built map"

**Solutions:**
1. Check that `map_path` is correct and accessible
2. Verify the map directory structure contains `graph.txt` and submap directories
3. Ensure the map was saved properly from a previous SLAM session
4. Check file permissions

### Issue: "No estimation frames available yet"

**Solutions:**
1. Wait a few seconds for the odometry system to initialize
2. Ensure sensor data (IMU and point clouds) are being received
3. Check that topics are correctly configured

### Issue: Relocalization fails or poor accuracy

**Solutions:**
1. Provide a more accurate initial pose estimate
2. Increase `linear_search_window` and `angular_search_window` in config
3. Decrease `min_localization_overlap` threshold (but not too low, e.g., 0.1)
4. Ensure the current environment matches the pre-built map
5. Check that the robot is within `max_localization_distance` of the map

### Issue: High memory usage

**Solutions:**
1. Reduce `num_keep_submaps` in config (e.g., from 3 to 2)
2. Disable `keep_raw_points` if enabled
3. Use lower resolution voxelmaps

## Performance Tips

1. **GPU Acceleration**: Use GPU-based registration for better performance
   ```json
   "registration_error_factor_type": "VGICP_GPU"
   ```

2. **Optimize Search Windows**: Adjust search windows based on your accuracy needs
   - Larger windows = more robust but slower
   - Smaller windows = faster but requires better initial pose

3. **Submap Management**: Balance memory vs accuracy with `num_keep_submaps`

4. **Sensor Quality**: Better sensor data (denser point clouds, accurate IMU) = better localization

## Advanced: Hybrid Mode

You can enable both mapping and localization simultaneously by loading a pre-built map while still creating new submaps. This allows:
- Localization against known areas
- Mapping of new areas
- Automatic loop closure between old and new submaps

This is the default behavior when `localization_mode` is enabled with `enable_local_mapping: true`.

## Code References

For implementation details, see:
- [glim_ros.hpp:48-56](/home/hungvt14/glim_ws/src/glim_ros2/include/glim_ros/glim_ros.hpp#L48-L56) - Localization callback declarations
- [glim_ros.cpp:141-197](/home/hungvt14/glim_ws/src/glim_ros2/src/glim_ros/glim_ros.cpp#L141-L197) - Localization mode initialization
- [glim_ros.cpp:467-559](/home/hungvt14/glim_ws/src/glim_ros2/src/glim_ros/glim_ros.cpp#L467-L559) - Relocalization callback implementations

## References

- GLIM Paper: [Link to paper if available]
- Main README: [README.md](README.md)
- Localization Class: `glim_localization/include/glim/mapping/localization.hpp`
