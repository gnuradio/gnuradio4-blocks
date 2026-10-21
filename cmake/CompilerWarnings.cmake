# from here:
#
# https://github.com/lefticus/cppbestpractices/blob/master/02-Use_the_Tools_Available.md

function(set_project_warnings target_name)

  # GCC 14 does not apply a header's diagnostic pragma to a warning raised after inlining when the header comes from a
  # precompiled header. algorithm/fourier/fft.hpp keeps -Wnull-dereference out of the vendored PocketFFT with such a
  # pragma. A system include does not cover a warning raised after inlining. A target whose precompiled header includes
  # algorithm/fourier/fft.hpp sets GR_PCH_INCLUDES_POCKETFFT. Under GCC 14 that target compiles without
  # -Wnull-dereference. GCC 15 and later keep the pragma through a precompiled header; the gate goes when GCC 14 leaves
  # the supported compilers.
  set(GCC14_PCH_POCKETFFT
      "$<AND:$<COMPILE_LANG_AND_ID:CXX,GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,15>,$<BOOL:$<TARGET_PROPERTY:GR_PCH_INCLUDES_POCKETFFT>>,$<BOOL:$<TARGET_PROPERTY:PRECOMPILE_HEADERS>>,$<NOT:$<BOOL:$<TARGET_PROPERTY:DISABLE_PRECOMPILE_HEADERS>>>>"
  )
  set(NO_NULL_DEREFERENCE
      "$<OR:$<AND:$<BOOL:$<TARGET_PROPERTY:GR_QA_REDUCED_OPTIMIZATION>>,$<NOT:$<CONFIG:Debug>>>,${GCC14_PCH_POCKETFFT}>")

  set(MSVC_WARNINGS
      /W4 # Baseline reasonable warnings
      /w14242 # 'identifier': conversion from 'type1' to 'type1', possible loss of data
      /w14254 # 'operator': conversion from 'type1:field_bits' to 'type2:field_bits', possible loss of data
      /w14263 # 'function': member function does not override any base class virtual member function
      /w14265 # 'classname': class has virtual functions, but destructor is not virtual instances of this class may not
              # be destructed correctly
      /w14287 # 'operator': unsigned/negative constant mismatch
      /we4289 # nonstandard extension used: 'variable': loop control variable declared in the for-loop is used outside
              # the for-loop scope
      /w14296 # 'operator': expression is always 'boolean_value'
      /w14311 # 'variable': pointer truncation from 'type1' to 'type2'
      /w14545 # expression before comma evaluates to a function which is missing an argument list
      /w14546 # function call before comma missing argument list
      /w14547 # 'operator': operator before comma has no effect; expected operator with side-effect
      /w14549 # 'operator': operator before comma has no effect; did you intend 'operator'?
      /w14555 # expression has no effect; expected expression with side- effect
      /w14619 # pragma warning: there is no warning number 'number'
      /w14640 # Enable warning on thread un-safe static member initialization
      /w14826 # Conversion from 'type1' to 'type_2' is sign-extended. This may cause unexpected runtime behavior.
      /w14905 # wide string literal cast to 'LPSTR'
      /w14906 # string literal cast to 'LPWSTR'
      /w14928 # illegal copy-initialization; more than one user-defined conversion has been implicitly applied
      /permissive- # standards conformance mode for MSVC compiler.
  )

  set(CLANG_WARNINGS
      -Wall
      -Wextra # reasonable and standard
      -Wshadow # warn the user if a variable declaration shadows one from a parent context
      -Wnon-virtual-dtor # warn the user if a class with virtual functions has a non-virtual destructor. This helps
                         # catch hard to track down memory errors
      -Wold-style-cast # warn for c-style casts
      -Wcast-align # warn for potential performance problem casts
      -Wunused # warn on anything being unused
      -Woverloaded-virtual # warn if you overload (not override) a virtual function
      -Wpedantic # warn if non-standard C++ is used
      -Wconversion # warn on type conversions that may lose data
      -Wsign-conversion # warn on sign conversions
      # warn if a null dereference is detected; unreliable below -O2, so reduced-optimization test targets suppress it
      # explicitly rather than by omission, which also turns it off when an installed SDK exports it as a usage
      # requirement; GCC 14 targets whose precompiled header includes PocketFFT also suppress it (see the top of this
      # function)
      $<$<NOT:${NO_NULL_DEREFERENCE}>:-Wnull-dereference>
      $<${NO_NULL_DEREFERENCE}:-Wno-null-dereference>
      -Wdouble-promotion # warn if float is implicit promoted to double
      -Wformat=2 # warn on security issues around functions that format output (ie printf)
      -Wno-unknown-pragmas # ignore IDE, GCC/CLANG specific pragmas
      -Wno-unused-command-line-argument # don't report extra arguments to the compiler
  )

  if(WARNINGS_AS_ERRORS)
    # Avoid warnings since they are often indicative of immature APIs and/or potential sources of bugs.
    set(CLANG_WARNINGS ${CLANG_WARNINGS} -Werror)
    set(MSVC_WARNINGS ${MSVC_WARNINGS} /WX)
    message(STATUS "Setting compiler warning as errors: ${WARNINGS_AS_ERRORS}")
  endif()

  # GCC 14 can report a false out-of-bounds copy in libstdc++'s inlined string code when a unit reads the shared test
  # precompiled header. It reports the copy under -Warray-bounds and, with that off, under -Wstringop-overflow. The same
  # unit compiles clean from the plain headers and under GCC 15 and later.
  set(GCC14_SHARED_PCH "$<AND:$<BOOL:$<TARGET_PROPERTY:GR_QA_SHARED_PCH>>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,15>>")

  set(GCC_EXTRA_WARNINGS
      -Wno-dangling-reference # TODO: remove this once the fmt dangling reference bug is fixed
      -Wmisleading-indentation # warn if indentation implies blocks where blocks do not exist
      -Wduplicated-cond # warn if if / else chain has duplicated conditions
      -Wduplicated-branches # warn if if / else branches have duplicated code
      -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
      -Wuseless-cast # warn if you perform a cast to the same type
      -Wno-interference-size # suppress ABI compatibility warnings for hardware inferred size
      -Wno-maybe-uninitialized # false positives if asan is enabled:
                               # https://gcc.gnu.org/bugzilla//show_bug.cgi?id=1056h6
      -Wno-tautological-compare # fmt has always true comparisons
      $<${GCC14_SHARED_PCH}:-Wno-array-bounds>
      $<${GCC14_SHARED_PCH}:-Wno-stringop-overflow>
      -fconcepts-diagnostics-depth=3
      -Wno-missing-field-initializers # confusing warning which is not what most users expect:
                                      # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=96868#c3
  )
  set(GCC_WARNINGS ${CLANG_WARNINGS} ${GCC_EXTRA_WARNINGS})

  if(MSVC)
    set(PROJECT_WARNINGS ${MSVC_WARNINGS})
  elseif(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    set(PROJECT_WARNINGS ${CLANG_WARNINGS})
  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(PROJECT_WARNINGS ${GCC_WARNINGS})
  else()
    message(AUTHOR_WARNING "No compiler warnings set for '${CMAKE_CXX_COMPILER_ID}' compiler.")
  endif()

  # Replace semicolons with spaces in PROJECT_WARNINGS
  string(
    REPLACE ";"
            " "
            PROJECT_WARNINGS_MOD
            "${PROJECT_WARNINGS}")
  set(ALL_COMPILER_FLAGS
      "${ALL_COMPILER_FLAGS}${PROJECT_WARNINGS_MOD}"
      PARENT_SCOPE)

  # the warning battery is this project's policy, not a usage requirement — the $<BUILD_INTERFACE:...> guard keeps it
  # out of the installed export so that find_package() consumers do not inherit -Wconversion/-Werror & co. via
  # INTERFACE_COMPILE_OPTIONS. Compiler-specific flags additionally stay behind compiler-id generator expressions rather
  # than the configure-time selection above, so in-tree builds mixing compilers (e.g. a Clang TU in a GCC-configured
  # build) never receive foreign flags such as -fconcepts-diagnostics-depth=3.
  target_compile_options(
    ${target_name}
    INTERFACE "$<BUILD_INTERFACE:$<$<CXX_COMPILER_ID:MSVC>:${MSVC_WARNINGS}>>"
              "$<BUILD_INTERFACE:$<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang,GNU>:${CLANG_WARNINGS}>>"
              "$<BUILD_INTERFACE:$<$<COMPILE_LANG_AND_ID:CXX,GNU>:${GCC_EXTRA_WARNINGS}>>")

endfunction()
