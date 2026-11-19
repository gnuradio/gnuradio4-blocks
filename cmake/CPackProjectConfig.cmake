# cpack includes this file (CPACK_PROJECT_CONFIG_FILE) once for each generator it runs, before it installs anything
# into its staging directory. Checks that concern only packaging belong here, so that a configure without packaging
# never meets them.

if(CPACK_GENERATOR STREQUAL "INNOSETUP")
  # Inno Setup takes each component name as written, and its compiler rejects any name that is not letters, digits and
  # underscores, or that is one of a few reserved words. Without this check a bad name fails only once ISCC runs, after
  # staging has finished.
  foreach(_gr_component IN LISTS CPACK_COMPONENTS_ALL)
    if(NOT _gr_component MATCHES "^[A-Za-z_][A-Za-z0-9_]*$" OR _gr_component MATCHES "^(not|and|or)$")
      message(FATAL_ERROR "install component '${_gr_component}' is not a valid Inno Setup component name: "
                          "use letters, digits and underscores, not starting with a digit")
    endif()
  endforeach()
endif()
