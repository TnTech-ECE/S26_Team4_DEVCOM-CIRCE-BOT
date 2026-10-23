cmake_minimum_required(VERSION 3.8)
project(circe_diagnostics)

find_package(ament_cmake REQUIRED)

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

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# PROGRAMS, not FILES: it has to land executable in lib/${PROJECT_NAME} for
# `ros2 run circe_diagnostics health_monitor.py` to find it. Same pattern as
# circe_perception.
install(
  PROGRAMS
    scripts/health_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
