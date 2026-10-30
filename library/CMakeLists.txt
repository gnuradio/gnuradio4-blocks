# The library: reusable algorithms that the blocks build on. Its targets link core and each other, never a block target
# (cmake/LibraryLayering.cmake). The parent directory provides the shared setup: core's package, Boost.UT, the warning
# options, httplib, the test helpers and add_benchmark.

# the FileIo HTTP client path needs cpr; the build leaves it off
set(GR_HTTP_ENABLED OFF)

if(NOT TARGET exprtk)
  add_library(exprtk STATIC third_party/exprtk.cpp)
  set_target_properties(exprtk PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(exprtk PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
                                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_compile_options(exprtk PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wno-null-dereference>)
  # this target does not link gnuradio-options, so it has to ask for /bigobj itself; the single
  # 46k-line ExprTk translation unit exceeds the COFF section limit without it (C1128)
  target_compile_options(exprtk PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)
endif()
if(NOT TARGET gnuradio4::exprtk)
  add_library(gnuradio4::exprtk ALIAS exprtk)
endif()

# The library installs its own package, gnuradio4Library, beside the blocks' package, so a consumer that finds
# gnuradio4Library and links gnuradio4::gnuradio-algorithm or gnuradio4::exprtk builds unchanged.
set(GR4_EXPORT_NAME gnuradio4LibraryTargets)
set(GR4_CMAKE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/gnuradio4Library")

add_subdirectory(algorithm)

include(cmake/LibraryLayering.cmake)
cmake_language(
  EVAL
  CODE
  "cmake_language(DEFER DIRECTORY [[${PROJECT_SOURCE_DIR}]] CALL gr_library_check_link_direction [[${CMAKE_CURRENT_SOURCE_DIR}]] [[${PROJECT_SOURCE_DIR}/blocks]])"
)

if(ENABLE_TESTING)
  add_test(NAME qa_LibraryLayering COMMAND ${CMAKE_COMMAND} -DSOURCE_ROOT=${PROJECT_SOURCE_DIR} -P
                                           ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckLibraryIncludes.cmake)
endif()

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gnuradio4LibraryConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4LibraryConfig.cmake" INSTALL_DESTINATION "${GR4_CMAKE_INSTALL_DIR}")

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4LibraryConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion)

configure_file(cmake/gnuradio4Library.pc.in "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4Library.pc" @ONLY)

# Every file the library installs is development material: headers, a static library and package files. Every rule here
# and in algorithm/ belongs to the Devel component, the component name core uses for its block-library generator files.
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4LibraryConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4LibraryConfigVersion.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4Library.pc"
  DESTINATION "${GR4_CMAKE_INSTALL_DIR}"
  COMPONENT Devel)

install(
  TARGETS exprtk
  EXPORT ${GR4_EXPORT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Devel)
install(
  FILES $<TARGET_FILE:exprtk>
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  COMPONENT Devel)
install(
  FILES third_party/exprtk.hpp
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT Devel)
install(
  DIRECTORY third_party/exprtk
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT Devel)

install(
  EXPORT ${GR4_EXPORT_NAME}
  NAMESPACE gnuradio4::
  DESTINATION "${GR4_CMAKE_INSTALL_DIR}"
  COMPONENT Devel)
