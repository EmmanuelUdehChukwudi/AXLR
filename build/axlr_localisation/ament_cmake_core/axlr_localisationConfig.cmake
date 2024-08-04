# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_axlr_localisation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED axlr_localisation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(axlr_localisation_FOUND FALSE)
  elseif(NOT axlr_localisation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(axlr_localisation_FOUND FALSE)
  endif()
  return()
endif()
set(_axlr_localisation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT axlr_localisation_FIND_QUIETLY)
  message(STATUS "Found axlr_localisation: 0.0.0 (${axlr_localisation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'axlr_localisation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${axlr_localisation_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(axlr_localisation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${axlr_localisation_DIR}/${_extra}")
endforeach()
