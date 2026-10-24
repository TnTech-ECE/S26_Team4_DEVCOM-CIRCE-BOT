cmake_minimum_required(VERSION 3.8)
project(circe_bringup)

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

install(
  DIRECTORY launch config udev
  DESTINATION share/${PROJECT_NAME}
)

# PROGRAMS, not FILES: wait_for.py has to land executable in lib/${PROJECT_NAME} for
# launch_ros to resolve `executable='wait_for.py'`, and for
# `ros2 run circe_bringup wait_for.py` to work when you want to test a readiness check
# by hand. Same pattern as circe_perception's scripts.
#
# Every launch file in this package chains its nodes through this script, so a build
# that skips this line produces a bringup that dies immediately with
# "executable 'wait_for.py' not found on the libexec directory".
#
# smoke_test.sh is installed too, so the robot can run `ros2 run circe_bringup
# smoke_test.sh hw` from its install space without a source checkout. Its simulation
# scenarios skip themselves where circe_gazebo is not installed, so this adds no
# dependency on it.
install(
  PROGRAMS
    scripts/wait_for.py
    scripts/smoke_test.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
