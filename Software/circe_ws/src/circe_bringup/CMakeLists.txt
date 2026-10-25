cmake_minimum_required(VERSION 3.8)
project(circe_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config udev
  DESTINATION share/${PROJECT_NAME}
)

# Installed as executables so launch files and `ros2 run` can find them.
install(
  PROGRAMS
    scripts/wait_for.py
    scripts/smoke_test.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
