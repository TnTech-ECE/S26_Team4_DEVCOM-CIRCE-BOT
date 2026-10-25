cmake_minimum_required(VERSION 3.8)
project(circe_teensy)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/cpplint; firmware is checked by PlatformIO, not the C++ linters.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Teensy firmware (skipped if pio isn't found) ---
option(CIRCE_TEENSY_BUILD_FIRMWARE "Build the Teensy firmware with PlatformIO" ON)
find_program(PIO_EXECUTABLE NAMES pio platformio
  HINTS "$ENV{HOME}/.platformio/penv/bin")

if(CIRCE_TEENSY_BUILD_FIRMWARE AND PIO_EXECUTABLE)
  # Build in colcon's build/ tree, not the source .pio/.
  set(PIO_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/pio")
  set(FIRMWARE_HEX "${PIO_BUILD_DIR}/teensy40/firmware.hex")

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

install(
  PROGRAMS
    scripts/teensy_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
