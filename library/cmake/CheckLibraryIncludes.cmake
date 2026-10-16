# Fails when a file under library/ includes a header of a block family. Two forms count: a header under a family's
# directory, gnuradio-4.0/<family>/, with the families read from blocks/*/include/gnuradio-4.0/; and a family's
# generated header, gnuradio-4.0/Gr<Family>Blocks.hpp, whose <Family> matches a blocks/*/ directory name in any letter
# case. third_party/ holds vendored code and is not read.
if(NOT DEFINED SOURCE_ROOT)
  message(FATAL_ERROR "SOURCE_ROOT must identify the gnuradio4-blocks source directory")
endif()

file(
  GLOB family_dirs
  LIST_DIRECTORIES true
  "${SOURCE_ROOT}/blocks/*/include/gnuradio-4.0/*")
set(families "")
foreach(dir IN LISTS family_dirs)
  if(IS_DIRECTORY "${dir}")
    get_filename_component(family "${dir}" NAME)
    list(APPEND families "${family}")
  endif()
endforeach()
list(REMOVE_DUPLICATES families)
if(NOT families)
  message(FATAL_ERROR "no block family include directory found under ${SOURCE_ROOT}/blocks")
endif()
file(
  GLOB block_entries
  LIST_DIRECTORIES true
  "${SOURCE_ROOT}/blocks/*")
set(block_dirs "")
foreach(entry IN LISTS block_entries)
  if(IS_DIRECTORY "${entry}")
    get_filename_component(name "${entry}" NAME)
    string(TOLOWER "${name}" name)
    list(APPEND block_dirs "${name}")
  endif()
endforeach()

list(
  JOIN
  families
  "|"
  family_pattern)

file(
  GLOB_RECURSE
  library_files
  "${SOURCE_ROOT}/library/*.h"
  "${SOURCE_ROOT}/library/*.hh"
  "${SOURCE_ROOT}/library/*.hpp"
  "${SOURCE_ROOT}/library/*.cc"
  "${SOURCE_ROOT}/library/*.cpp")
list(
  FILTER
  library_files
  EXCLUDE
  REGEX
  "/library/third_party/")

set(errors "")
foreach(file IN LISTS library_files)
  file(STRINGS "${file}" includes REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]gnuradio-4\\.0/(${family_pattern})/")
  foreach(line IN LISTS includes)
    string(STRIP "${line}" line)
    string(APPEND errors "\n  ${file}: ${line}")
  endforeach()
  file(STRINGS "${file}" generated REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]gnuradio-4\\.0/Gr[A-Za-z0-9_]*Blocks")
  foreach(line IN LISTS generated)
    string(
      REGEX
      REPLACE ".*gnuradio-4\\.0/Gr([A-Za-z0-9_]*)Blocks.*"
              "\\1"
              name
              "${line}")
    string(TOLOWER "${name}" name)
    if(name IN_LIST block_dirs)
      string(STRIP "${line}" line)
      string(APPEND errors "\n  ${file}: ${line}")
    endif()
  endforeach()
endforeach()

list(LENGTH library_files file_count)
list(LENGTH families family_count)
if(errors)
  message(FATAL_ERROR "library files include block headers; the library may use core and itself only:${errors}")
endif()
message(STATUS "${file_count} library files include no header of the ${family_count} block families")
