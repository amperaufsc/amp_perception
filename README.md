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

# ROS Interfaces

## Topics (LiDAR)

| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| LiDAR     | Sub       | `/sub_pointcloud`                   | `sensor_msgs/PointCloud2`     | PointCloud input |
| LiDAR     | Sub       | `/sub_image`                   | `sensor_msgs/Image`     | Image input |
| LiDAR     | Sub       | `/pub_pointcloud`                   | `sensor_msgs/PointCloud2`     | PointCloud output |

> Topics and messages used in LiDAR package.

---

## Topics (Perception)

| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Perception     | Pub       | `/camera/rgb/image_raw`                   | `sensor_msgs/Image`     | Image Output |
| Perception     | Pub       | `/disparity_msg`                   | `estereo_msgs/DisparityImage`     | Disparity Output |
| Perception     | Pub       | `/track`                   | `fsds_msgs/Track`     | Track Output |
| Perception     | Pub       | `/cone`                   | `fsds_msgs/Cone`     | Cone msg Output |
| Perception     | Pub       | `/point_clound`                   | `sensor_msgs/PointCloud2`     | PointCloud2 Output |
| Perception     | Pub       | `/camera/left/image_raw`                   | `sensor_msgs/Image`     | Left camera image |
| Perception     | Pub       | `/camera/right/image_raw`                   | `sensor_msgs/Image`     | Right camera image |


> Topics and messages used in Perception package.

---

## Topics (Yolo)

| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Yolo     | Sub       | `/Yolov8_Inference`                   | `yolov8_msgs/Yolov8Inference`     | Inference input |
| Yolo     | Sub       | `/rgb_cam/image_raw`                   | `sensor_msgs/Image`     | Image input |
| Yolo     | Pub       | `/inference_result_cv2`                   | `sensor_msgs/Image`     | Inference output |
| Yolo     | Pub       | `/inferenceimg`                   | `sensor_msgs/Image`     | Image output |

> Topics and messages used in Yolo package.

---

# Dependencies

Core dependencies (minimum):

- ROS 2 Humble (or newer)
- `rclcpp` / `rclpy`
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `lifecycle_msgs`
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
    colcon build --packages-select perception
   ```
```bash
    colcon build --packages-select lidar_filtering
   ```


## Running & Launching

### LiDAR launchs: 

```bash
    ros2 run lidar_filtering lidar_fusion.py
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
