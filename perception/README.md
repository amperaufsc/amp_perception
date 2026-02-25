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

## ROS Interfaces

### Topics (Perception)

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

## Dependencies

Core dependencies (minimum):

- ROS 2 Humble (or newer)
- `rclcpp` / `rclpy`
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`
- `tf2` + `tf2_ros`
- `colcon` (build system)

---

### For compiling, use: 

```bash
    colcon build --packages-select perception
   ```

### For launch, use: 

```bash
    ros2 run perception depthai_camera_publisher.py
   ```

```bash
    ros2 launch perception amp_depthai.launch.py
   ```
## Setup camera Luxonis OAK-D-LR

### Dependencies
  To install the necessary dependencies, simply run `OAK_D_LR_Setup`. Do not forget to check the name of your system's ROS 2 workspace; by default, it is set to ws. If that is not your case, change it.

  *OAK_D_LR_Setup:*
```bash
    cd ws/src/
    git clone --branch humble https://github.com/luxonis/depthai-ros.git
    cd ..
    sudo apt update
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    source /opt/ros/humble/setup.bash
    MAKEFLAGS="-j1 -l1" colcon build
    source install/setup.bash
```
  
  After installing `OAK_D_LR_Setup`, simply run the launcher to start the camera and publish the topics

  *Camera launcher:*
```bash
    source install/setup.bash	
    ros2 launch depthai_ros_driver camera.launch.py 
```
#### Possible lauch error
If you get the error 'Insufficient permissions to communicate with X_LINK_BOOTLOADER device with name "3.3". Make sure udev rules are set' when running the launcher, it means the USB port is blocked. If this happens, cancel the launch, run the codes below, and then disconnect and reconnect the USB cable.

```bash
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
```

#### Source: https://docs.luxonis.com/software/ros/depthai-ros/build/

### Parameter
  Parameters are essential for operation, defining, for example, which topics will be published or the specific configurations under which the camera will operate. To change them, simply navigate to the `/ws/src/depthai-ros/depthai_ros_driver/config` package and modify the `camera.yaml` file.
*These are the base parameters used by Ampera.*
```yaml
    /**:
    ros__parameters:
      camera:
        i_enable_imu: true
        i_enable_ir: true
        i_nn_type: none
        i_pipeline_type: RGBD
      pipeline_gen:
        i_enable_imu: true
      imu:
        i_message_type: IMU
        i_enable_rotation: true
        i_acc_freq: 400
        i_gyro_freq: 400
        i_rot_freq: 400
      rgb:
        i_disable_node: true
        i_simulate_from_topic: true
        i_publish_topic: false
      left:
        i_publish_topic: true
        i_fps: 60.0
        i_resolution: 1200P
      right:
        i_publish_topic: true
        i_fps: 60.0
        i_resolution: 1200P
      stereo:
        i_depth_preset: HIGH_DENSITY
        i_disparity_width: DISPARITY_96
        i_align_depth: true
        i_board_socket_id: 1
        i_extended_disp: true
        i_subpixel: true
        i_subpixel_fractional_bits: 5
        i_lr_check: true
        i_lrc_threshold: 5
        i_max_q_size: 4
        i_enable_brightness_filter: false
        i_enable_decimation_filter: false
        i_enable_spatial_filter: false
        i_enable_speckle_filter: false
        i_enable_temporal_filter: false
        i_stereo_conf_threshold: 240
        i_publish_topic: true
```

#### Source and Parameters: https://docs.luxonis.com/software/ros/depthai-ros/driver/
