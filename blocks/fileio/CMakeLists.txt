add_library(gr-fileio INTERFACE)
add_library(gnuradio4::gr-fileio ALIAS gr-fileio)
target_link_libraries(gr-fileio INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-fileio INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                               $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-fileio EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/fileio"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

set(GrFileIoBlocks_HDRS include/gnuradio-4.0/fileio/BasicFileIo.hpp include/gnuradio-4.0/fileio/WavBlocks.hpp)
if(WIN32)
  # FdSource and FdSink are built on POSIX descriptors and poll(), which Windows does not provide. Naming the header keeps
  # the registration check on for the rest of the module; only a core whose block-library macro declares the check takes
  # the keyword.
  if(DEFINED GR_BLOCKLIB_REQUIRE_REGISTERED_HEADERS)
    set(GrFileIoBlocks_UNREGISTERED_ARGS UNREGISTERED_HEADERS include/gnuradio-4.0/fileio/FdIo.hpp)
  endif()
else()
  list(APPEND GrFileIoBlocks_HDRS include/gnuradio-4.0/fileio/FdIo.hpp)
endif()

gr_add_block_library(
  GrFileIoBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  SPLIT_BLOCK_INSTANTIATIONS
  HEADERS
  ${GrFileIoBlocks_HDRS}
  ${GrFileIoBlocks_UNREGISTERED_ARGS}
  LINK_LIBRARIES
  gnuradio4::gr-fileio)

if(TARGET gnuradio4::GrFileIoBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()
