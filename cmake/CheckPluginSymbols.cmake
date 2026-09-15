if(NOT DEFINED PLUGIN_ROOT)
  message(FATAL_ERROR "PLUGIN_ROOT must identify the directory holding the built block libraries")
endif()

if(NOT DEFINED NM_EXECUTABLE)
  message(FATAL_ERROR "NM_EXECUTABLE must name the nm binary that reads the dynamic symbol tables")
endif()

if(NOT DEFINED LIBRARY_SUFFIX)
  set(LIBRARY_SUFFIX ".so")
endif()

# Every registration unit the block-library generator writes defines one gr_blocklib_init_unit_ function, and the module
# integrator calls each of them. A unit the integrator calls but the library does not compile leaves that symbol
# undefined, and the library fails to load with no earlier signal: it links as a shared object, and only dlopen reports
# the miss.
set(kInitUnitSymbol "gr_blocklib_init_unit_[A-Za-z0-9_]+")

file(GLOB_RECURSE block_libraries "${PLUGIN_ROOT}/*${LIBRARY_SUFFIX}")

set(swept_libraries 0)
set(defined_units 0)
set(errors "")

foreach(library IN LISTS block_libraries)
  execute_process(
    COMMAND ${NM_EXECUTABLE} -D --undefined-only "${library}"
    RESULT_VARIABLE nm_result
    OUTPUT_VARIABLE undefined_symbols
    ERROR_VARIABLE nm_error
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_STRIP_TRAILING_WHITESPACE)

  if(NOT
     nm_result
     EQUAL
     0)
    string(APPEND errors "\n  ${library}: could not be read: ${nm_error}")
    continue()
  endif()

  execute_process(
    COMMAND ${NM_EXECUTABLE} -D --defined-only "${library}"
    OUTPUT_VARIABLE defined_symbols
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

  string(
    REGEX MATCHALL
          "${kInitUnitSymbol}"
          resident_units
          "${defined_symbols}")
  list(LENGTH resident_units resident_count)
  math(EXPR defined_units "${defined_units} + ${resident_count}")

  string(
    REGEX MATCHALL
          "${kInitUnitSymbol}"
          missing_units
          "${undefined_symbols}")
  if(missing_units)
    list(REMOVE_DUPLICATES missing_units)
    string(
      REPLACE ";"
              ", "
              missing_units
              "${missing_units}")
    string(APPEND errors "\n  ${library}: calls registration units it does not carry: ${missing_units}")
  endif()

  math(EXPR swept_libraries "${swept_libraries} + 1")
endforeach()

# Both counts guard against a sweep that inspects nothing and reports success for it
if(swept_libraries EQUAL 0)
  message(FATAL_ERROR "No shared libraries were found under ${PLUGIN_ROOT}")
endif()

if(defined_units EQUAL 0)
  message(FATAL_ERROR "None of the ${swept_libraries} libraries under ${PLUGIN_ROOT} carries a block registration unit")
endif()

if(errors)
  message(FATAL_ERROR "Block libraries with unresolved registration units:${errors}")
endif()

message(STATUS "Swept ${swept_libraries} shared libraries carrying ${defined_units} block registration units")
