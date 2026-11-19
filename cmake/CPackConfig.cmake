set(CPACK_PACKAGE_VENDOR "GNU Radio")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "GNU Radio 4 blocks")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
# without this CPack substitutes its own placeholder licence into the installer
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_CONTACT "admin@gnuradio.org")
if(NOT CPACK_PACKAGE_NAME)
  set(CPACK_PACKAGE_NAME "gnuradio4-blocks")
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")

include(CPackComponent)

# The components are whatever this tree's install() rules name, read once they
# have all run: this file is included at the end of the top-level CMakeLists.
# A list kept here by hand silently drops a component it does not name. That
# happened once, and the installer shipped gnuradio4Blocks without the
# gnuradio4Library it depends on.
get_cmake_property(GR_BLOCKS_CPACK_COMPONENTS COMPONENTS)
set(CPACK_COMPONENTS_ALL ${GR_BLOCKS_CPACK_COMPONENTS})

# Unspecified holds the block headers, the block libraries and plugins, and
# the gnuradio4Blocks package. Devel holds the algorithm library: its headers,
# exprtk and the gnuradio4Library package. gnuradio4Blocks finds
# gnuradio4Library, so neither is usable alone, and both are REQUIRED.
# REQUIRED is also what makes Unspecified safe as macOS's REGISTER_COMPONENT.
cpack_add_component(
  Unspecified
  DISPLAY_NAME "Blocks"
  DESCRIPTION "GNU Radio 4 block families: headers, libraries, plugins and the gnuradio4Blocks CMake package"
  REQUIRED)
cpack_add_component(
  Devel
  DISPLAY_NAME "Algorithm library"
  DESCRIPTION "The GNU Radio 4 algorithm library the blocks are built on, and its gnuradio4Library CMake package"
  REQUIRED)
cpack_add_component(gr4_example_graphs DISPLAY_NAME "Example graphs"
                    DESCRIPTION "GRC example flowgraphs, installed under share/gnuradio-4.0/examples")

# APPLE implies UNIX, so macOS must be excluded here or it selects the Debian
# generator and `--target package` tries to build a .deb on a Mac.
if(UNIX AND NOT APPLE)
  if(NOT CPACK_GENERATOR)
    set(CPACK_GENERATOR "DEB")
  endif()

  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
  set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
  # When installing to a non-standard prefix (e.g. /opt/gnuradio4), dpkg-shlibdeps
  # cannot find the package's own private libraries. Point it to the install lib dir.
  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS_PRIVATE_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
  if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(CPACK_DEBIAN_DEBUGINFO_PACKAGE ON)
  endif()
endif()

if(APPLE)
  set(CPACK_GENERATOR "productbuild")
  if(NOT COMMAND gr_configure_productbuild_installer)
    message(FATAL_ERROR "GR_PACKAGING on macOS needs a gnuradio4 core that installs GrProductBuild.cmake")
  endif()
  # gr_configure_productbuild_installer() comes from gr-core's installed
  # GrProductBuild.cmake, which gnuradio4Config.cmake includes on APPLE.
  #
  # REGISTER_COMPONENT carries the manifest registration, so it must be one
  # that is always installed: Unspecified, which is REQUIRED above.

  # the identifier identifies gr-blocks to pkgutil across releases: never reuse
  # it in another GR installer and never change it between versions
  gr_configure_productbuild_installer(
    IDENTIFIER
    "org.gnuradio.gr4.blocks"
    PACKAGES
    "gr-blocks=${CPACK_PACKAGE_VERSION}"
    REGISTER_COMPONENT
    Unspecified
    PRESELECTED_COMPONENTS
    ${CPACK_COMPONENTS_ALL})
endif()

if(WIN32)
  set(CPACK_GENERATOR "INNOSETUP")
  if(NOT COMMAND gr_configure_innosetup_installer)
    message(FATAL_ERROR "GR_PACKAGING on Windows needs a gnuradio4 core that installs GrInnoSetup.cmake")
  endif()
  # cpack checks the component names against Inno Setup's rules before it
  # stages anything; see CPackProjectConfig.cmake
  set(CPACK_PROJECT_CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/CPackProjectConfig.cmake")
  set(CPACK_INNOSETUP_USE_CMAKE_BOOL_FORMAT ON)
  set(CPACK_INNOSETUP_USE_MODERN_WIZARD ON)
  set(CPACK_INNOSETUP_ALLOW_CUSTOM_DIRECTORY ON)
  # fallback only: gr_configure_innosetup_installer() takes the install root from
  # the shared manifest when a GR 4 installation already exists
  set(CPACK_INNOSETUP_INSTALL_ROOT "{autopf}")
  set(CPACK_PACKAGE_INSTALL_DIRECTORY "GNURadio4")
  # The Start Menu folder is GR_START_MENU, defaulted and applied by
  # gr_configure_innosetup_installer() below so that every GR 4 package agrees
  # on it. Setting it here would win and pin the old, unversioned name, which
  # would put this package's shortcuts in a folder of their own.

  # gr_configure_innosetup_installer() comes from find_package(gnuradio4)
  # the AppId identifies gr-blocks to Windows across releases: never reuse it in
  # another GR installer and never change it between versions
  gr_configure_innosetup_installer(
    APP_ID
    "{{8aa11ca1-4756-4280-b07b-e9aea3a00358}"
    PACKAGES
    "gr-blocks=${CPACK_PACKAGE_VERSION}"
    PRESELECTED_COMPONENTS
    ${CPACK_COMPONENTS_ALL})
endif()

include(CPack)
