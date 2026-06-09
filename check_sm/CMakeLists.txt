cmake_minimum_required(VERSION 3.8)
project(check_sm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smacc2 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ros_timer_client REQUIRED)
find_package(keyboard_client REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(check_sm_node src/main.cpp)

target_include_directories(check_sm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(check_sm_node
  rclcpp
  smacc2
  rclcpp_lifecycle
  lifecycle_msgs
  ros_timer_client
  keyboard_client
  std_msgs
)

install(
        PROGRAMS
        can_sim_node.py
        DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS check_sm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()