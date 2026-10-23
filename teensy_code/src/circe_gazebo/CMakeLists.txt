cmake_minimum_required(VERSION 3.8)
project(circe_gazebo)

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

# Without this the launch/, worlds/, config/ and rviz/ directories are never installed
# and `ros2 launch circe_gazebo ...` fails with "file not found" even after a successful
# build. ros2 pkg create does not add it.
install(
  DIRECTORY launch worlds config rviz
  DESTINATION share/${PROJECT_NAME}
)

# PROGRAMS, not FILES: stop_sim.sh has to land executable so it can be run straight out
# of the install space. See the script's own header for why killing Gazebo needs care -
# on Harmonic the server runs as `ruby gz sim server`, so the obvious `pkill -f gz-sim`
# misses it and the next launch spawns a second rover into the surviving world.
install(
  PROGRAMS scripts/stop_sim.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
