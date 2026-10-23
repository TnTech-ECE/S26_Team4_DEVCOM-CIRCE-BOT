cmake_minimum_required(VERSION 3.8)
project(circe_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Without this the launch/, config/, rviz/ and maps/ directories are never installed
# and `ros2 launch circe_nav ...` fails with "file not found" even after a successful
# build. ros2 pkg create does not add it.
# PROGRAMS, not FILES: has to land executable in lib/${PROJECT_NAME} for
# `ros2 run circe_nav cable_mission.py` to find it. Same pattern as circe_perception.
install(
  PROGRAMS
    scripts/cable_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
