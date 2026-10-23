# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_circe_diagnostics_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED circe_diagnostics_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(circe_diagnostics_FOUND FALSE)
  elseif(NOT circe_diagnostics_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(circe_diagnostics_FOUND FALSE)
  endif()
  return()
endif()
set(_circe_diagnostics_CONFIG_INCLUDED TRUE)

# output package information
if(NOT circe_diagnostics_FIND_QUIETLY)
  message(STATUS "Found circe_diagnostics: 0.1.0 (${circe_diagnostics_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'circe_diagnostics' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT circe_diagnostics_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(circe_diagnostics_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${circe_diagnostics_DIR}/${_extra}")
endforeach()
