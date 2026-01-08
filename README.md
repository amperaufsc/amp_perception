### Perception 
This repository groups the perception packages responsible for cone detection within the ROS 2 stack. These packages process raw sensor data and publish cone detections as structured ROS 2 messages for downstream modules (e.g., mapping and path planning).

The perception modules are designed to:

Acquire data from onboard sensors (e.g. cameras, depth sensors, or LiDAR)

Detect cones in the environment

Output detections with timestamps and frame consistency

Publish ROS 2 topics in standard reference frames

Provide spatial information suitable for further processing

The repository standardizes detection outputs at the ROS 2 interface level, ensuring compatibility between perception and higher-level modules without embedding planning or decision logic.
