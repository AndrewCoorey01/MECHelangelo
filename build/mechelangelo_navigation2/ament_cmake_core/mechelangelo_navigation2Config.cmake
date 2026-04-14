# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mechelangelo_navigation2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mechelangelo_navigation2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mechelangelo_navigation2_FOUND FALSE)
  elseif(NOT mechelangelo_navigation2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mechelangelo_navigation2_FOUND FALSE)
  endif()
  return()
endif()
set(_mechelangelo_navigation2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mechelangelo_navigation2_FIND_QUIETLY)
  message(STATUS "Found mechelangelo_navigation2: 2.3.6 (${mechelangelo_navigation2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mechelangelo_navigation2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mechelangelo_navigation2_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mechelangelo_navigation2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mechelangelo_navigation2_DIR}/${_extra}")
endforeach()
