cmake_minimum_required (VERSION 3.10)
project (ORD53)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

if ( NOT EXISTS "${PROJECT_SOURCE_DIR}/surf/easyloggingpp/src/easylogging++.h")
  message( FATAL_ERROR "We are missing easylogging in surf/easyloggingpp/.  See the README.md file for details." )
endif()

execute_process(COMMAND git describe --abbrev=8 --dirty --always --tags
                OUTPUT_VARIABLE GIT_REV
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET)
if ("${GIT_REV}" STREQUAL "")
  SET(GIT_REV "git:N/A")
endif()
SET(SURFVERSION "1.99")
SET(VERSIONGIT "${SURFVERSION}-${GIT_REV}")

set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_STANDARD 17)

if ( NOT MSVC )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall" )

  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -W -Wfloat-equal -Wundef -Wpointer-arith" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wwrite-strings" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wredundant-decls -Wchar-subscripts -Wcomment -Wformat=2" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wwrite-strings -Wmissing-declarations -Wredundant-decls" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wswitch-enum" )

  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Winit-self -Wmissing-field-initializers" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Waddress -Wmissing-noreturn -Wstrict-overflow=1" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wextra -Warray-bounds" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wshadow" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pedantic" )

  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Werror=switch-enum" )
  # set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Werror=shadow" )
  # set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Werror=float-equal" )

### XXX REMOVE EVENTUALLY
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-unused-parameter" )
  set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-unused-variable" )
endif()

if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Debug CACHE STRING
       "Choose the type of build, options are: Debug Release."
       FORCE )
endif()
set (CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE 1)

string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPERCASE_CMAKE_BUILD_TYPE)

if (NOT UPPERCASE_CMAKE_BUILD_TYPE MATCHES "^(DEBUG|RELEASE)$")
  message(FATAL_ERROR "Invalid value for CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
endif()

# so we can run in valgrind
add_definitions(-DCGAL_DISABLE_ROUNDING_MATH_CHECK=ON)

add_definitions(-DELPP_NO_DEFAULT_LOG_FILE)


if(UPPERCASE_CMAKE_BUILD_TYPE MATCHES DEBUG)
  SET (DEBUG_OUTPUT_DEFAULT ON)
  SET (DEBUG_COLLAPSE_TIMES ON)
  SET (SURF_NDEBUG OFF)
else()
  SET (DEBUG_OUTPUT_DEFAULT OFF)
  SET (DEBUG_COLLAPSE_TIMES OFF)
  SET (SURF_NDEBUG ON)
endif()

## Set default option values unless values given on the command line.
if (NOT DEFINED DEBUG_COLLAPSE_TIMES)
  SET (DEBUG_COLLAPSE_TIMES ${DEBUG_COLLAPSE_TIMES})
endif()
if (NOT DEFINED DEBUG_OUTPUT)
  SET (DEBUG_OUTPUT ${DEBUG_OUTPUT_DEFAULT})
endif()

# declare the options
option(REFINE_TRIANGULATION       "Refine triangulation (to avoid flip events)" ON)

# option(DEBUG_EXPENSIVE_PREDICATES "Check assertions that we expect to be expensive" OFF)
set         (DEBUG_EXPENSIVE_PREDICATES SOME CACHE STRING "Check assertions that we expect to be expensive")
set_property(CACHE DEBUG_EXPENSIVE_PREDICATES PROPERTY STRINGS SOME ON OFF)

if(UPPERCASE_CMAKE_BUILD_TYPE MATCHES DEBUG)
  if(DEBUG_EXPENSIVE_PREDICATES STREQUAL "SOME")
    set (DEBUG_EXPENSIVE_PREDICATES 1)
  elseif(DEBUG_EXPENSIVE_PREDICATES)
    set (DEBUG_EXPENSIVE_PREDICATES 2)
  else()
    set (DEBUG_EXPENSIVE_PREDICATES OFF)
  endif()
else()
  set (DEBUG_EXPENSIVE_PREDICATES OFF)
endif()

option(TEST_SUITE                 "Build test suite" OFF)

option(DEBUG_COLLAPSE_TIMES       "Double check collapse times and add extra safeties" ${DEBUG_COLLAPSE_TIMES})

option(DEBUG_OUTPUT               "Include logging at debug level" ${DEBUG_OUTPUT})
option(DEBUG_OUTPUT_WITH_FILES    "Include filenames and line numbersin debug output" OFF)

option(NT_USE_DOUBLE              "Use double as the number type" OFF)
option(HEAP_STATS                 "Gather stats on heap properties" OFF)

option(BUILD_SHARED_LIBS          "Build surflib as a shared library" OFF)
option(LIB_ONLY                   "Build surflib only, no gui or cli" OFF)

configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  @ONLY
)
include_directories("${PROJECT_BINARY_DIR}")

if(LIB_ONLY)
  subdirs (surf)
else()
  subdirs (surf gui cc)
endif()

enable_testing()
