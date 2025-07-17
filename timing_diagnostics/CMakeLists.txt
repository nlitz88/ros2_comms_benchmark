cmake_minimum_required(VERSION 3.8)
project(timing_diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Define a library for the event timing diagnostic task.
add_library(event_timing_diagnostic_task SHARED src/event_timing_diagnostic_task.cpp)

# Define the packages that the library depends on.
ament_target_dependencies(
  event_timing_diagnostic_task
  rclcpp
  diagnostic_updater
)

# Specify the include directories for the library.
target_include_directories(event_timing_diagnostic_task
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Do we need to separately install the include directory?
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library.
ament_export_targets(export_event_timing_diagnostic_task HAS_LIBRARY_TARGET)
install(TARGETS event_timing_diagnostic_task
        EXPORT export_event_timing_diagnostic_task
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Export the dependencies of this library.
ament_export_dependencies(diagnostic_updater)

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
