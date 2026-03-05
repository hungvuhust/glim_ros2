# GLIM ROS2 Launch Files Guide

This directory contains launch files for running glim_ros in different modes.

## Available Launch Files

### 1. `glim_ros.launch.py` - SLAM Mode
Launch file for standard SLAM (Simultaneous Localization and Mapping) mode.

**Basic Usage:**
```bash
ros2 launch glim_ros glim_ros.launch.py
```

**With Custom Parameters:**
```bash
ros2 launch glim_ros glim_ros.launch.py \
  config_path:=config/velodyne \
  params_file:=config/glim_ros_params.yaml \
  debug:=false \
  use_sim_time:=false
```

**Launch Arguments:**
- `config_path`: Path to glim config directory (default: `glim/config/velodyne`)
  - Can be absolute path or relative to glim package
- `params_file`: Path to ROS parameters YAML file (default: `glim_ros/config/glim_ros_params.yaml`)
  - Can be absolute path or relative to glim_ros package
- `debug`: Enable debug logging (default: `false`)
- `dump_on_unload`: Save map on shutdown (default: `false`)
- `use_sim_time`: Use simulation time (default: `false`)

**Example - Custom Config:**
```bash
ros2 launch glim_ros glim_ros.launch.py \
  config_path:=/home/user/my_glim_config \
  params_file:=config/my_custom_params.yaml
```

---

### 2. `glim_localization.launch.py` - Localization Mode
Launch file for localization mode against a pre-built map.

**Basic Usage:**
```bash
ros2 launch glim_ros glim_localization.launch.py \
  map_path:=/path/to/your/saved/map
```

**With Custom Parameters:**
```bash
ros2 launch glim_ros glim_localization.launch.py \
  map_path:=/home/user/maps/my_map \
  config_path:=config/velodyne \
  params_file:=config/glim_localization_params.yaml \
  debug:=false \
  use_sim_time:=false
```

**Launch Arguments:**
- `map_path`: **REQUIRED** - Path to pre-built map directory
  - Must contain map saved from previous SLAM run
- `config_path`: Path to glim config directory (default: `glim/config/velodyne`)
- `params_file`: Path to ROS parameters YAML file (default: `glim_ros/config/glim_localization_params.yaml`)
- `debug`: Enable debug logging (default: `false`)
- `use_sim_time`: Use simulation time (default: `false`)

**Example - From Bag File:**
```bash
# Terminal 1: Launch localization
ros2 launch glim_ros glim_localization.launch.py \
  map_path:=~/maps/office_building \
  use_sim_time:=true

# Terminal 2: Play rosbag
ros2 bag play my_data.db3 --clock
```

---

## Configuration Files

Configuration files are located in the `config/` directory:

### `glim_ros_params.yaml` - SLAM Mode Parameters
Default parameters for SLAM mode including:
- Sensor topics
- TF frames
- Extension modules
- Time offsets
- Debug settings

**Customize by copying:**
```bash
cp config/glim_ros_params.yaml config/my_params.yaml
# Edit my_params.yaml
ros2 launch glim_ros glim_ros.launch.py params_file:=config/my_params.yaml
```

### `glim_localization_params.yaml` - Localization Mode Parameters
Parameters optimized for localization mode including:
- Localization-specific settings
- Relocalization parameters
- Extension modules for localization viewer

---

## Typical Workflows

### Workflow 1: Create a Map (SLAM)

1. **Launch in SLAM mode:**
   ```bash
   ros2 launch glim_ros glim_ros.launch.py
   ```

2. **Drive the robot or play rosbag:**
   ```bash
   ros2 bag play data.db3
   ```

3. **Save the map when done:**
   ```bash
   # Method 1: Via service (if implemented)
   ros2 service call /glim_ros/save_map std_srvs/srv/Trigger

   # Method 2: Enable dump_on_unload
   ros2 launch glim_ros glim_ros.launch.py dump_on_unload:=true
   # Ctrl+C to shutdown and auto-save to /tmp/dump
   ```

### Workflow 2: Localize in Existing Map

1. **Launch in localization mode:**
   ```bash
   ros2 launch glim_ros glim_localization.launch.py \
     map_path:=/path/to/saved/map
   ```

2. **Set initial pose in RViz:**
   - Click "2D Pose Estimate" button
   - Click and drag on the map to set pose

3. **Robot will auto-relocalize and start tracking**

4. **Manual relocalization (if needed):**
   ```bash
   ros2 service call /glim_ros/trigger_relocalization std_srvs/srv/Trigger
   ```

### Workflow 3: Simulation with Use Sim Time

```bash
# Terminal 1: Launch with sim time
ros2 launch glim_ros glim_ros.launch.py use_sim_time:=true

# Terminal 2: Run simulator or play bag with clock
ros2 bag play data.db3 --clock
```

---

## Topic Remapping

To remap sensor topics without modifying config files, edit the launch file remappings section:

```python
remappings=[
    ('/imu', '/your_robot/imu'),
    ('/velodyne_points', '/your_robot/lidar'),
],
```

Or create a custom launch file that imports and extends the base launch files.

---

## Troubleshooting

### Issue: "No module named 'yaml'"
```bash
sudo apt install python3-yaml
```

### Issue: "Map path does not exist"
Verify the map path is correct and contains:
- `graph.txt`
- Submap directories (`000000/`, `000001/`, etc.)

### Issue: Sensor data not received
Check topic names match your configuration:
```bash
ros2 topic list
ros2 topic echo /imu
ros2 topic echo /velodyne_points
```

Update `config/glim_ros_params.yaml` or use launch file parameters.

---

## Advanced Usage

### Custom Launch File

Create your own launch file extending the base:

```python
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    glim_ros_dir = get_package_share_directory('glim_ros')

    glim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(glim_ros_dir, 'launch', 'glim_ros.launch.py')
        ),
        launch_arguments={
            'config_path': 'config/my_sensor',
            'debug': 'true',
        }.items()
    )

    # Add your custom nodes here

    return LaunchDescription([
        glim_launch,
        # your_custom_nodes,
    ])
```

### Multiple Robots

Use namespaces:
```bash
ros2 launch glim_ros glim_ros.launch.py \
  namespace:=robot1 \
  params_file:=config/robot1_params.yaml
```

---

## See Also

- [LOCALIZATION_MODE.md](../LOCALIZATION_MODE.md) - Detailed localization mode documentation
- [README.md](../README.md) - Main glim_ros documentation
- GLIM configuration files: `$(ros2 pkg prefix glim)/share/glim/config/`
