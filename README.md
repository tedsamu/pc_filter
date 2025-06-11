# pc_filter

Generic ROS 2 package for real-time filtering of 3D point clouds. The package subscribes to a PointCloud2 topic, applies configurable pass-through filtering, and republishes the filtered cloud.

## Features
- Pass-through filtering on x, y, or z fields
- Configurable filter field, min/max range, and negative filtering
- Written in Python, adheres to flake8 style

## Requirements
- ROS 2 Humble
- Python 3.8+
- numpy >=1.17.3,<1.25.0
- open3d >=0.13.0,<0.19.0
- sensor_msgs_py
- rclpy

## Installation
Clone the repository into your ROS 2 workspace `src` directory:

```bash
git clone https://github.com/<your-org>/pc_filter.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/pc_filter/requirements.txt
colcon build --packages-select pc_filter
```

## Usage
Launch the filter node, this example will pass through all points with a x value between 0 and 50:

```bash
ros2 run pc_filter pointcloud_filter_node \
    --ros-args \
    -p input_topic:=/lidar/points \
    -p output_topic:=/lidar/filtered \
    -p filter_field:=x \
    -p filter_min:=0.0 \
    -p filter_max:=50.0
```


Or include in your launch file. Parameters can be set via ROS 2 params or YAML:
- `input_topic` (string, default: `/lidar/points`)
- `output_topic` (string, default: `/lidar/filtered`)
- `filter_field` (string: `x`, `y`, or `z`)
- `filter_min` (float)
- `filter_max` (float)
- `filter_negative` (bool)

