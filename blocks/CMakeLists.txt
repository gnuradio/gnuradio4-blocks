set(GR4_BLOCK_STATIC_LIBRARY "")
if(EMSCRIPTEN)
  set(GR4_BLOCK_STATIC_LIBRARY MAKE_STATIC_LIBRARY)
endif()

add_subdirectory(common)
add_subdirectory(testing)
add_subdirectory(basic)
add_subdirectory(electrical)
add_subdirectory(fileio)
add_subdirectory(filter)
add_subdirectory(fourier)
add_subdirectory(http)
add_subdirectory(math)
add_subdirectory(timing)

# Optional families go before libs: the registry audit there links the shared library a family's target provides, so a
# family added after it is detected by the audit but not linked.
if(GR4_ENABLE_AUDIO)
  add_subdirectory(audio)
endif()

# shared library that contains all registered blocks
add_subdirectory(libs)

if(GR4_ENABLE_SDR)
  add_subdirectory(sdr)
endif()
