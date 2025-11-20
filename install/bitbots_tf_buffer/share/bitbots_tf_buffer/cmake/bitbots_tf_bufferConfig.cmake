# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bitbots_tf_buffer_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bitbots_tf_buffer_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bitbots_tf_buffer_FOUND FALSE)
  elseif(NOT bitbots_tf_buffer_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bitbots_tf_buffer_FOUND FALSE)
  endif()
  return()
endif()
set(_bitbots_tf_buffer_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bitbots_tf_buffer_FIND_QUIETLY)
  message(STATUS "Found bitbots_tf_buffer: 1.0.0 (${bitbots_tf_buffer_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bitbots_tf_buffer' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bitbots_tf_buffer_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bitbots_tf_buffer_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bitbots_tf_buffer_DIR}/${_extra}")
endforeach()
