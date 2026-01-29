cmake_minimum_required(VERSION 3.8)
project(lidar_filtering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs)
find_package(Open_CV)
find_package(yolov8_msgs REQUIRED)
find_package(fs_msgs REQUIRED)

# Inclui diretórios (OpenCV, ROS2, PCL, etc)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Executável C++
add_executable(lidar_fusion src/lidar_fusion.cpp)
add_executable(ground_segmentation src/ground_segmentation.cpp)
add_executable(fusion_image src/fusion_image.cpp)
add_executable(lidar_fusion_simulator src/lidar_fusion_simulator.cpp)
add_executable(fusion_image_simulator src/fusion_image_simulator.cpp)
# Link com bibliotecas do ROS 2 e OpenCV
ament_target_dependencies(lidar_fusion
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  OpenCV
  message_filters
  yolov8_msgs
  fs_msgs
)
ament_target_dependencies(ground_segmentation
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  std_msgs
)
ament_target_dependencies(fusion_image
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  OpenCV
  message_filters
  yolov8_msgs
)
ament_target_dependencies(lidar_fusion_simulator
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  OpenCV
  message_filters
  yolov8_msgs
  fs_msgs
)
ament_target_dependencies(fusion_image_simulator
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  OpenCV
  message_filters
  yolov8_msgs
)
target_link_libraries(lidar_fusion
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)
target_link_libraries(ground_segmentation
  ${PCL_LIBRARIES}
)
target_link_libraries(fusion_image
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)
target_link_libraries(lidar_fusion_simulator
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)
target_link_libraries(fusion_image_simulator
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
lidar_fusion
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
ground_segmentation
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
fusion_image
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
lidar_fusion_simulator
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
fusion_image_simulator
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()