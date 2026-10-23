cmake_minimum_required(VERSION 3.8)
project(circe_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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

# Without this the launch/ and config/ directories are never installed and
# `ros2 launch circe_perception ...` fails with "file not found" even after a successful
# build. ros2 pkg create does not add it.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# PROGRAMS, not FILES: this has to land executable in lib/${PROJECT_NAME} for
# `ros2 run circe_perception terrain_node.py` to find it. Same pattern as
# roverrobotics_input_manager, which installs scripts/joys_manager.py this way -- a Python
# node in an ament_cmake package is not a new convention in this workspace.
install(
  PROGRAMS
    scripts/terrain_node.py
    scripts/scan_pitch_filter.py
    scripts/imu_fusion.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
