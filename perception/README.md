# Perception

## Overview
This package is responsible for subscribing to left and right camera feeds and the disparity map. By integrating YOLO bounding box detections, it computes the 3D spatial coordinates of each cone. The resulting data is then aggregated and published as `sensor_msgs/PointCloud2` and `fs_msgs/Track` message types.

### Intrinsics matrix configuration/position_estimation class and baseline setting
  
#### Matrix data location
The intrinsic matrices used as the basis for the calculations in the `/amp_perception/perception/position_estimation/disparity_estimator.py` class and it depends on which sensor is being used, depending on camera model. These matrices are stored in YAML files in `/amp_perception/config`, which contain important information such as the focal length and optical center of both cameras, both left and right.

-> The matrix data is accessed in this line of disparity_estimator.py:

<img width="560" height="61" alt="image" src="https://github.com/user-attachments/assets/9e303f63-b8ce-4070-90a2-fd2991a7bbda" />

-> Intrinsics matrix structure:

<img width="251" height="170" alt="image" src="https://github.com/user-attachments/assets/282597a9-e168-46f3-b4a5-6426cb8f3c4d" />

#### Baseline setting
To set baseline, the key information for position estimation, that is the space between left and right camera sensor used in meters, can be changed at triangulation function of disparity_estimator.py.

<img width="804" height="304" alt="image" src="https://github.com/user-attachments/assets/cb1083b6-1ede-4bd3-b571-1c9f6b406826" />

---

# ROS Interfaces

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

---

# Dependencies

Core dependencies (minimum):

- ROS 2 Humble (or newer)
- `rclcpp` / `rclpy`
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`
- `tf2` + `tf2_ros`
- `colcon` (build system)

---

## For compiling, use: 

```bash
    colcon build --packages-select perception
   ```

## For launch, use: 

```bash
    ros2 run perception depthai_camera_publisher.py
   ```

```bash
    ros2 launch perception amp_depthai.launch.py
   ```
