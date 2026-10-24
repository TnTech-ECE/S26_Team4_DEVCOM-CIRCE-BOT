cmake_minimum_required(VERSION 3.8)
project(circe_teensy)

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
  # src/main.cpp is Arduino firmware, not ROS code: the C++ linters would flag
  # Arduino idioms and miss what matters. PlatformIO's compiler is its check.
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Teensy firmware ---------------------------------------------------------
# The VS Code PlatformIO extension installs pio into its own venv, not onto PATH.
# Off, or pio missing (e.g. the RViz laptop), skips the firmware and nothing else.
option(CIRCE_TEENSY_BUILD_FIRMWARE "Build the Teensy firmware with PlatformIO" ON)
find_program(PIO_EXECUTABLE NAMES pio platformio
  HINTS "$ENV{HOME}/.platformio/penv/bin")

if(CIRCE_TEENSY_BUILD_FIRMWARE AND PIO_EXECUTABLE)
  # Build into colcon's build/ tree rather than the source tree's .pio/, so a colcon
  # build and a VS Code build never fight over the same object files.
  set(PIO_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/pio")
  set(FIRMWARE_HEX "${PIO_BUILD_DIR}/teensy40/firmware.hex")

  # ALL with no tracked outputs, so it runs on every build; pio's own dependency
  # tracking makes an unchanged rebuild take a couple of seconds.
  add_custom_target(firmware ALL
    COMMAND ${CMAKE_COMMAND} -E env PLATFORMIO_BUILD_DIR=${PIO_BUILD_DIR}
      ${PIO_EXECUTABLE} run --project-dir ${CMAKE_CURRENT_SOURCE_DIR} --environment teensy40
    COMMENT "Building Teensy 4.0 firmware with PlatformIO"
    VERBATIM)

  install(FILES ${FIRMWARE_HEX} DESTINATION share/${PROJECT_NAME}/firmware)
elseif(CIRCE_TEENSY_BUILD_FIRMWARE)
  message(WARNING "PlatformIO not found: skipping the Teensy firmware. "
    "Install the VS Code PlatformIO extension, or pass "
    "-DCIRCE_TEENSY_BUILD_FIRMWARE=OFF to silence this.")
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# PROGRAMS, not FILES: it has to land executable in lib/${PROJECT_NAME} for
# `ros2 run circe_teensy teensy_bridge.py` to find it. Same pattern as
# circe_diagnostics.
install(
  PROGRAMS
    scripts/teensy_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
