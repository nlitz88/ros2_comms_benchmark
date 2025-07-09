cmake_minimum_required(VERSION 3.8)
project(comms_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Per https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-a-Composable-Node.html#cmake-changes
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(timing_diagnostics REQUIRED)

# Benchmark Publisher Component
add_library(benchmark_publisher_component SHARED src/benchmark_publisher.cpp)
ament_target_dependencies(
  benchmark_publisher_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  timing_diagnostics
)

target_include_directories(benchmark_publisher_component
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

rclcpp_components_register_node(
  benchmark_publisher_component
  PLUGIN "comms_benchmark::BenchmarkPublisher"
  EXECUTABLE benchmark_publisher
)

ament_export_targets(export_benchmark_publisher_component)
install(TARGETS benchmark_publisher_component
        EXPORT export_benchmark_publisher_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Benchmark Subscriber Component
add_library(benchmark_subscriber_component SHARED src/benchmark_subscriber.cpp)
ament_target_dependencies(
  benchmark_subscriber_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  timing_diagnostics
)

target_include_directories(benchmark_subscriber_component
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

rclcpp_components_register_node(
  benchmark_subscriber_component
  PLUGIN "comms_benchmark::BenchmarkSubscriber"
  EXECUTABLE benchmark_subscriber
)

ament_export_targets(export_benchmark_subscriber_component)
install(TARGETS benchmark_subscriber_component
        EXPORT export_benchmark_subscriber_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
