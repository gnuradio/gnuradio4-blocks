# A library target may link core targets and other library targets, and never a block target.
# gr_library_check_link_direction fails the configure when a target defined under LIBRARY_DIR reaches a block target
# through LINK_LIBRARIES or INTERFACE_LINK_LIBRARIES. A block target is a target defined under BLOCKS_DIR or a name of
# the block-target form (gr-<family>, gnuradio4::gr-<family>, Gr<Family>Blocks...). The check follows the
# INTERFACE_LINK_LIBRARIES of every non-imported target outside LIBRARY_DIR that a library target links, so a wrapper
# target cannot hide a block target. A name in a generator expression counts as well; a file path does not. The check
# needs every link of the project in place, so it runs at the end of the top-level directory. The include direction has
# its own test, qa_LibraryLayering.
function(gr_library_check_link_direction LIBRARY_DIR BLOCKS_DIR)
  set(_targets "")
  set(_directories "${LIBRARY_DIR}")
  while(_directories)
    list(POP_FRONT _directories _directory)
    get_property(
      _directoryTargets
      DIRECTORY "${_directory}"
      PROPERTY BUILDSYSTEM_TARGETS)
    get_property(
      _subdirectories
      DIRECTORY "${_directory}"
      PROPERTY SUBDIRECTORIES)
    list(APPEND _targets ${_directoryTargets})
    list(APPEND _directories ${_subdirectories})
  endwhile()

  set(_errors "")
  foreach(_target IN LISTS _targets)
    foreach(_property IN ITEMS LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
      get_target_property(_items ${_target} ${_property})
      if(NOT _items)
        continue()
      endif()
      # each entry of the work list is one linked name and the chain of wrapper targets it was reached through
      set(_work "")
      foreach(_item IN LISTS _items)
        string(
          REGEX MATCHALL
                "[A-Za-z0-9_.+/-]+(::[A-Za-z0-9_.+-]+)?"
                _names
                "${_item}")
        foreach(_name IN LISTS _names)
          list(APPEND _work "${_name}|")
        endforeach()
      endforeach()
      set(_visited "")
      while(_work)
        list(POP_FRONT _work _entry)
        string(FIND "${_entry}" "|" _bar REVERSE)
        string(
          SUBSTRING "${_entry}"
                    0
                    ${_bar}
                    _name)
        math(EXPR _chainStart "${_bar} + 1")
        string(
          SUBSTRING "${_entry}"
                    ${_chainStart}
                    -1
                    _chain)
        if(_name MATCHES "/" OR _name IN_LIST _visited)
          continue()
        endif()
        list(APPEND _visited "${_name}")
        set(_blocksTarget OFF)
        if(_name MATCHES "^(gnuradio4::)?gr-" OR _name MATCHES "^Gr[A-Za-z0-9_]*Blocks")
          set(_blocksTarget ON)
        elseif(TARGET "${_name}")
          get_target_property(_resolved "${_name}" ALIASED_TARGET)
          if(NOT _resolved)
            set(_resolved "${_name}")
          endif()
          get_target_property(_imported "${_resolved}" IMPORTED)
          if(NOT _imported)
            get_target_property(_sourceDir "${_resolved}" SOURCE_DIR)
            cmake_path(
              IS_PREFIX
              BLOCKS_DIR
              "${_sourceDir}"
              NORMALIZE
              _blocksTarget)
            cmake_path(
              IS_PREFIX
              LIBRARY_DIR
              "${_sourceDir}"
              NORMALIZE
              _inLibrary)
            if(NOT _blocksTarget AND NOT _inLibrary)
              get_target_property(_wrapped "${_resolved}" INTERFACE_LINK_LIBRARIES)
              if(_wrapped)
                string(
                  REGEX MATCHALL
                        "[A-Za-z0-9_.+/-]+(::[A-Za-z0-9_.+-]+)?"
                        _wrappedNames
                        "${_wrapped}")
                foreach(_wrappedName IN LISTS _wrappedNames)
                  list(APPEND _work "${_wrappedName}|${_chain}${_name} -> ")
                endforeach()
              endif()
            endif()
          endif()
        endif()
        if(_blocksTarget)
          string(APPEND _errors "\n  ${_target} ${_property}: ${_chain}${_name}")
        endif()
      endwhile()
    endforeach()
  endforeach()

  list(LENGTH _targets _count)
  if(_errors)
    message(FATAL_ERROR "library targets link block targets; the library may use core and itself only:${_errors}")
  endif()
  message(STATUS "Library layering: ${_count} library targets link no block target")
endfunction()
