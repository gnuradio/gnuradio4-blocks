# Findlibsoundio
#
# Defines libsoundio::libsoundio. libsoundio ships no package of its own, so this looks for one in three places, in
# order:
#
# 1. a CMake or CPS package (gr-deps installs both), which defines the target itself
# 2. pkg-config, whose include and library directories and version guide the searches below
# 3. a plain search for soundio/soundio.h and the soundio library
#
# soundio.h carries no version, so a copy found by the plain search alone has none and satisfies any version request.

include(FindPackageHandleStandardArgs)

set(_libsoundio_version_args "")
if(libsoundio_FIND_VERSION_RANGE)
  set(_libsoundio_version_args "${libsoundio_FIND_VERSION_RANGE}")
elseif(libsoundio_FIND_VERSION)
  set(_libsoundio_version_args "${libsoundio_FIND_VERSION}")
  if(libsoundio_FIND_VERSION_EXACT)
    list(APPEND _libsoundio_version_args EXACT)
  endif()
endif()

# a package that was found but rejected (its version, say) is still the answer: falling through would let the searches
# below accept the same library with no version to check
find_package(
  libsoundio
  ${_libsoundio_version_args}
  CONFIG
  QUIET)
unset(_libsoundio_version_args)
if(libsoundio_FOUND OR libsoundio_CONSIDERED_CONFIGS)
  find_package_handle_standard_args(libsoundio CONFIG_MODE)
  return()
endif()

unset(libsoundio_VERSION)
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_libsoundio QUIET libsoundio)
  if(PC_libsoundio_FOUND)
    set(libsoundio_VERSION "${PC_libsoundio_VERSION}")
  endif()
endif()

find_path(
  libsoundio_INCLUDE_DIR
  NAMES soundio/soundio.h
  HINTS ${PC_libsoundio_INCLUDEDIR} ${PC_libsoundio_INCLUDE_DIRS})
find_library(
  libsoundio_LIBRARY
  NAMES soundio
  HINTS ${PC_libsoundio_LIBDIR} ${PC_libsoundio_LIBRARY_DIRS})
mark_as_advanced(libsoundio_INCLUDE_DIR libsoundio_LIBRARY)

find_package_handle_standard_args(
  libsoundio
  REQUIRED_VARS libsoundio_LIBRARY libsoundio_INCLUDE_DIR
  VERSION_VAR libsoundio_VERSION HANDLE_VERSION_RANGE)

if(libsoundio_FOUND AND NOT TARGET libsoundio::libsoundio)
  add_library(libsoundio::libsoundio UNKNOWN IMPORTED)
  set_target_properties(libsoundio::libsoundio PROPERTIES IMPORTED_LOCATION "${libsoundio_LIBRARY}"
                                                          INTERFACE_INCLUDE_DIRECTORIES "${libsoundio_INCLUDE_DIR}")
endif()
