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

# Inclui diretórios (OpenCV, ROS2, PCL, etc)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Executável C++
add_executable(lidar_fusion src/lidar_fusion.cpp)
add_executable(ground_segmentation src/ground_segmentation.cpp)

# Link com bibliotecas do ROS 2 e OpenCV
ament_target_dependencies(lidar_fusion
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  OpenCV
  message_filters
)
ament_target_dependencies(ground_segmentation
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  std_msgs
)

target_link_libraries(lidar_fusion
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)
target_link_libraries(ground_segmentation
  ${PCL_LIBRARIES}
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

ament_package()
