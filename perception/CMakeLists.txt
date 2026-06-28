cmake_minimum_required(VERSION 3.8)
project(perception)

# Default to C++14 if no standard is specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yolov8_msgs REQUIRED)
#find_package(fs_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
        PROGRAMS
        lifecycles/disparity_pub_life.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS
        lifecycles/track_node_pub_life.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS
        lifecycles/perception_lifecycle_node.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS
        utils/img_descompress.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS
        utils/capture_images.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS
        utils/conecomparisonnode.py
        DESTINATION lib/${PROJECT_NAME}
)
install(
        PROGRAMS
        utils/disparity_map_pub.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS
        utils/perception_calc.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS
        utils/cone_track_node.py
        DESTINATION lib/${PROJECT_NAME}
)

#install(
#        PROGRAMS
#        ros2/position_estimator.py
#        DESTINATION lib/${PROJECT_NAME}
#)

install(
        PROGRAMS
        ros2/depthai_position_estimator.py
        DESTINATION lib/${PROJECT_NAME}
)

#install(
#        PROGRAMS
#        ros2/test_node.py
#        DESTINATION lib/${PROJECT_NAME}
#)

# install(
#         PROGRAMS
#         ros2/camera_info_pub.py
#         DESTINATION lib/${PROJECT_NAME}
# )

install(
        PROGRAMS
        ros2/lifecycle_dpe.py
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY
        utils
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY
        stereo_camera/
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY
        position_estimation
        DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS
        position_estimation/stereo_pipeline.py
        position_estimation/keypoints_estimator.py
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
