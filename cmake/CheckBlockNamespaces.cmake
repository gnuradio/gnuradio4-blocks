if(NOT DEFINED SOURCE_ROOT)
  message(FATAL_ERROR "SOURCE_ROOT must identify the gnuradio4-blocks source directory")
endif()

file(GLOB_RECURSE block_headers "${SOURCE_ROOT}/blocks/*/include/*.h" "${SOURCE_ROOT}/blocks/*/include/*.hh"
     "${SOURCE_ROOT}/blocks/*/include/*.hpp")

set(registration_count 0)
set(errors "")

foreach(header IN LISTS block_headers)
  file(READ "${header}" contents)
  string(REGEX MATCHALL "GR_REGISTER_BLOCK\\([^\n\r]+\\)" registrations "${contents}")
  if(NOT registrations)
    continue()
  endif()

  string(REGEX MATCH "/blocks/([^/]+)/include/" path_match "${header}")
  set(module "${CMAKE_MATCH_1}")
  set(expected_prefix "gr::${module}::")

  foreach(registration IN LISTS registrations)
    math(EXPR registration_count "${registration_count} + 1")
    string(REGEX REPLACE "^GR_REGISTER_BLOCK\\((.*)\\)$" "\\1" arguments "${registration}")
    string(STRIP "${arguments}" arguments)

    if(arguments MATCHES "^\"([^\"]+)\"[ \t]*,[ \t]*([^,\\)]+)")
      set(alias "${CMAKE_MATCH_1}")
      set(block_type "${CMAKE_MATCH_2}")
      string(STRIP "${block_type}" block_type)
      if(NOT alias MATCHES "^${expected_prefix}")
        string(APPEND errors "\n  ${header}: alias '${alias}' must start with '${expected_prefix}'")
      endif()
    else()
      string(REGEX MATCH "^[^,\\)]+" block_type "${arguments}")
      string(STRIP "${block_type}" block_type)
    endif()

    if(NOT block_type MATCHES "^${expected_prefix}")
      string(APPEND errors "\n  ${header}: block type '${block_type}' must start with '${expected_prefix}'")
    endif()
  endforeach()
endforeach()

if(errors)
  message(FATAL_ERROR "GR_REGISTER_BLOCK namespace inconsistencies:${errors}")
endif()

message(STATUS "Validated ${registration_count} GR_REGISTER_BLOCK declarations")
