# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_r1_arm_hardware_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED r1_arm_hardware_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(r1_arm_hardware_FOUND FALSE)
  elseif(NOT r1_arm_hardware_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(r1_arm_hardware_FOUND FALSE)
  endif()
  return()
endif()
set(_r1_arm_hardware_CONFIG_INCLUDED TRUE)

# output package information
if(NOT r1_arm_hardware_FIND_QUIETLY)
  message(STATUS "Found r1_arm_hardware: 0.0.0 (${r1_arm_hardware_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'r1_arm_hardware' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${r1_arm_hardware_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(r1_arm_hardware_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${r1_arm_hardware_DIR}/${_extra}")
endforeach()
