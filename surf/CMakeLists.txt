project (surf VERSION "${SURFVERSION}" DESCRIPTION "surflib")

INCLUDE (CheckTypeSize)


if ( NOT MSVC )
  set_source_files_properties(easyloggingpp/src/easylogging++.cc PROPERTIES COMPILE_FLAGS "-Wno-float-equal -Wno-missing-noreturn")
endif()

find_package( CGAL COMPONENTS Core )
if ( NOT CGAL_FOUND )
  message(STATUS "CGAL not found.")
  return()
endif()
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}. CGAL_USE_FILE is ${CGAL_USE_FILE}" )
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}." )
set ( CGAL_BUILD_TYPE_INIT ${CMAKE_BUILD_TYPE} )
include( ${CGAL_USE_FILE} )

find_package( Boost REQUIRED graph iostreams)
if ( NOT Boost_FOUND )
  message(STATUS "Boost not found.")
  return()
endif()

#find_package(MPFI)
#if ( NOT MPFR_FOUND )
#  message(STATUS "MPFI not found.")
#  return()
#endif()
#include(${MPFI_USE_FILE})

#include( CGAL_CreateSingleSourceCGALProgram )

include_directories(${Boost_INCLUDE_DIRS})

# COMPILER SETTINGS

if ( NOT MSVC )
  # set( CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -Werror" )
endif()

# BINARIES AND SOURCES

# cause easylogging to pretty please don't print to stdout.

add_library(surflib
  src/cgaltools.cpp
  src/BGLGraph.cpp
  src/SkeletonDCEL.cpp
  src/SkeletonStructure.cpp
  src/BasicInput.cpp
  src/BasicTriangulation.cpp
  src/EventQueue.cpp
  src/KineticTriangulation.cpp
  src/KineticTriangle.cpp
  src/WavefrontEdge.cpp
  src/WavefrontVertex.cpp
  src/CollapseSpec.cpp
  src/tools.cpp
  src/surf.cpp
  easyloggingpp/src/easylogging++.cc
  )
set_target_properties(surflib PROPERTIES VERSION ${PROJECT_VERSION})

target_link_libraries(surflib ${Boost_LIBRARIES})
target_link_libraries(surflib ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

set_target_properties(surflib PROPERTIES PUBLIC_HEADER include/tools.h)

target_compile_definitions(surflib PRIVATE ELPP_CUSTOM_COUT=std::cerr)

target_include_directories(surflib PRIVATE include)
target_include_directories(surflib PRIVATE src)

try_compile(HAVE_INTERSECTION_OF_CONSTRAINTS_EXCEPTION
  "${CMAKE_BINARY_DIR}/temp"
  SOURCES "${PROJECT_SOURCE_DIR}/../cmake/tests/test_CGAL__Intersection_of_constraints_exception.cpp"
  CMAKE_FLAGS "-DCMAKE_CXX_LINK_EXECUTABLE='echo not linking now...'" )

# also checks for existance and puts it into HAVE_...
set(CMAKE_EXTRA_INCLUDE_FILES "CGAL/Constrained_triangulation_2.h")
check_type_size("CGAL::No_constraint_intersection_tag" CGAL_NO_CONSTRAINT_INTERSECTION_TAG
                 LANGUAGE CXX)
set(CMAKE_EXTRA_INCLUDE_FILES)


configure_file (
  "${PROJECT_SOURCE_DIR}/surfconfig.h.in"
  "${PROJECT_BINARY_DIR}/surfconfig.h"
  @ONLY
)

include(GNUInstallDirs)

#### TESTING ##
if(TEST_SUITE)
  set( TEST_BINARY_NAME runtests )
  include(GoogleTest)
  add_executable( ${TEST_BINARY_NAME}
    test/TestBGLGraph.cpp
    test/TestHeap.cpp
    src/BGLGraph.cpp
    src/surf.cpp
    easyloggingpp/src/easylogging++.cc
    )
  gtest_add_tests(TARGET ${TEST_BINARY_NAME})

  if ( NOT MSVC )
    set_target_properties(${TEST_BINARY_NAME} PROPERTIES PRIVATE "-Wnofloat-equal")
  endif()
  target_include_directories(${TEST_BINARY_NAME} PRIVATE include)
  target_include_directories(${TEST_BINARY_NAME} PRIVATE src)
  target_include_directories(${TEST_BINARY_NAME} PRIVATE test)
  target_link_libraries(${TEST_BINARY_NAME} gtest gtest_main pthread )
  target_link_libraries(${TEST_BINARY_NAME} ${Boost_LIBRARIES} )
  target_link_libraries(${TEST_BINARY_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

  if ( NOT MSVC )
    target_compile_options(${TEST_BINARY_NAME} PRIVATE "-Wno-unused-function")
  endif()
ENDIF()
