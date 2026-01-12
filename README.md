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

self.rgb_publisher = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.disparity_map = self.create_publisher(DisparityImage, '/disparity_msg', 10)
        self.detection_publisher = self.create_publisher(Track, '/track', 10)
        self.position_publisher = self.create_publisher(Cone, '/cone', 10) 
        self.publishers_point_clound = self.create_publisher(PointCloud2, '/point_clound',10)
        self.monoLeft_publisher = self.create_publisher(Image, '/camera/left/image_raw',10)
        self.monoRight_publisher = self.create_publisher(Image, '/camera/right/image_raw',10)

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
| Path Planning     | Sub       | `/odom`                   | `sensor_msgs/Odom`     | Odometry input |
| Path Planning     | Sub       | `/mission/go_signal`      | `std_msgs/Bool`               | Trigger for planning |
| Path Planning     | Sub       | `/track`      | `nav_msgs/Track`               | Track input |
| Path Planning     | Pub       | `/path`  | `fsds_msgs/Path`      | Reference path |
| Path Planning     | Pub       | `/path_concatenated`  | `nav_msgs/Path`      | Reference path (Control input) |
| Path Planning     | Pub       | `/track_pointcloud`  | `nav_msgs/PointCloud2`      | Track for debugging |

> Topics and messages used in Yolo package.

---


# Coordinate Frames

Common frames in use:

- `map` – global SLAM / mapping frame
- `/fsds/map` – fsds frame 
- `base_link` – vehicle base frame (control reference)

Frame transforms are managed through TF2.

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
    colcon build --packages-select ros2_path_planning
   ```
```bash
    colcon build --packages-select ros2_control
   ```


## Running & Launching

### Path Planning launchs: 

```bash
    ros2 run ros2_path_planning path_node.py
   ```

```bash
    ros2 launch ros2_path_planning path_planning.launch.py
   ```

### Control launchs: 

```bash
    ros2 run ros2_control control_node.py
   ```

```bash
    ros2 launch ros2_control control.launch.py
   ```
