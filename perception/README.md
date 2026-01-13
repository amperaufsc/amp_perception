# Perception

## Overview
  This package is responsible for subscribing to left and right camera feeds and the disparity map. By integrating YOLO bounding box detections, it computes the 3D spatial coordinates of each cone. The resulting data is then aggregated and published as `sensor_msgs/PointCloud2` and `fs_msgs/Track` message types.

### Intrinsics matrix configuration, disparity_estimator class and baseline setting
  
#### Matrix data location
  The intrinsic matrices used as the basis for the calculations in the `/amp_perception/perception/position_estimation/disparity_estimator.py` class and it depends on which sensor is being used, depending on camera model. These matrices are stored in YAML files in `/amp_perception/config`, which contain important information such as the focal length and optical center of both cameras, both left and right.

-> The matrix data is accessed in this line of disparity_estimator.py:

<img width="560" height="61" alt="image" src="https://github.com/user-attachments/assets/9e303f63-b8ce-4070-90a2-fd2991a7bbda" />
<img width="432" height="106" alt="image" src="https://github.com/user-attachments/assets/b79733eb-0564-4c7b-b02b-41714b7abd90" />

-> Intrinsics matrix structure:

<img width="251" height="170" alt="image" src="https://github.com/user-attachments/assets/282597a9-e168-46f3-b4a5-6426cb8f3c4d" />

#### The disparity_estimator algorithm
  After receiving the YOLO bounding box message — which consists of two (x,y) points that demarcate a window in which the identified object is located —, the bounding box is clearly applied to the disparity map — in which each point of the image represents the difference in pixels of a specific point in common between the left and right images —, obtaining a list with the various disparity values from that window. In this way, the average (d) of these values is calculated — in order to minimize noise — and used in the base calculation of the points in real space `Z_Position = (Baseline*Focal_Length)/d`. After the disparity process, X and Y points are extracted based on Z.
  
<img width="415" height="465" alt="image" src="https://github.com/user-attachments/assets/4fb2262c-ae63-4530-8543-1279fb227383" />
<img width="411" height="462" alt="image" src="https://github.com/user-attachments/assets/50eb1cf4-8428-42af-aa2a-c8d088048800" />
<img width="831" height="142" alt="image" src="https://github.com/user-attachments/assets/eb18e7e4-f1b3-4a43-8137-ed40ae24de05" />
<img width="472" height="241" alt="image" src="https://github.com/user-attachments/assets/917a84c7-84b4-48b4-abf2-8d53eb2505ce" />


#### Baseline setting
  To set the baseline -the key information for position estimation-, that is the space between left and right camera sensors used in meters, can be changed in the `triangulation` function of disparity_estimator.py.

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
