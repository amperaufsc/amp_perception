# Perception 
This repository groups the perception packages responsible for cone detection within the ROS 2 stack. These packages process raw sensor data and publish cone detections as structured ROS 2 messages for downstream modules (e.g., mapping and path planning).

## The perception modules are designed to:
→ Acquire data from onboard sensors (e.g. cameras, depth sensors, or LiDAR)
→ Detect cones in the environment
→ Output detections with timestamps and frame consistency
→ Publish ROS 2 topics in standard reference frames
→ Provide spatial information suitable for further processing

The repository standardizes detection outputs at the ROS 2 interface level, ensuring compatibility between perception and higher-level modules without embedding planning or decision logic.

---

## Topics (LiDAR)

| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| LiDAR     | Sub       | `/sub_pointcloud`                   | `sensor_msgs/PointCloud2`     | PointCloud input |
| LiDAR     | Sub       | `/sub_image`                   | `sensor_msgs/Image`     | Image input |
| LiDAR     | Sub       | `/pub_pointcloud`                   | `sensor_msgs/PointCloud2`     | PointCloud output |

> Topics and messages used in LiDAR package.

---


# Dependencies

Core dependencies (minimum):

- ROS 2 Humble (or newer)
- `rclcpp` / `rclpy`
- `yolov8_msgs`, `geometry_msgs`, `sensor_msgs`, `lifecycle_msgs`, `fs_msgs`, `opencv`, `pcl`
- `tf2` + `tf2_ros`
- `colcon` (build system)

---

## Commands for compiling packages 

### For compiling both, use: 
```bash
    colcon build 
   ```

### For compiling individualy, use: 
```bash
    colcon build --packages-select yolobot_recognition
   ```
```bash
    colcon build --packages-select yolov8_msgs
   ```

```bash
    colcon build --packages-select fs_msgs
   ```
```bash
    colcon build --packages-select lidar_filtering
   ```


## Running & Launching

### LiDAR launchs: 

```bash
    ros2 run lidar_filtering lidar_fusion.cpp
   ```

```bash
    ros2 launch lidar_filtering camera_lidar.launch.py
   ```

### Perception launchs: 

```bash
    ros2 run perception depthai_camera_publisher.py
   ```

```bash
    ros2 launch perception amp_depthai.launch.py
   ```

### Yolo launchs: 

```bash
    ros2 run yolobot_recognition yolov8_ros2_pt.py
   ```

```bash
    ros2 launch yolobot_recognition launch_yolov8.launch.py
   ```
