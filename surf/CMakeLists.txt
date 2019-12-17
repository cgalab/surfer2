project (surf VERSION "${SURFVERSION}" DESCRIPTION "surflib")

set_source_files_properties(easyloggingpp/src/easylogging++.cc PROPERTIES COMPILE_FLAGS "-Wno-float-equal -Wno-missing-noreturn")

find_package( CGAL COMPONENTS Core )
if ( NOT CGAL_FOUND )
  message(STATUS "CGAL not found.")
  return()
endif()
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}. CGAL_USE_FILE is ${CGAL_USE_FILE}" )
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}." )
set ( CGAL_BUILD_TYPE_INIT ${CMAKE_BUILD_TYPE} )
include( ${CGAL_USE_FILE} )

find_package( Boost REQUIRED graph)
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

#set( CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -Werror" )
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

configure_file (
  "${PROJECT_SOURCE_DIR}/surfconfig.h.in"
  "${PROJECT_BINARY_DIR}/surfconfig.h"
  @ONLY
)

include(GNUInstallDirs)

#### TESTING ##
#include_directories("${PROJECT_SOURCE_DIR}")

include(ExternalProject)
ExternalProject_Add(
    googletest
    DOWNLOAD_COMMAND ""
    SOURCE_DIR "/usr/src/googletest/googletest"
    PREFIX "googletest"
    INSTALL_COMMAND ""
    LOG_CONFIGURE ON
    LOG_BUILD ON
)
ExternalProject_Get_Property(googletest binary_dir)
add_library(gtest UNKNOWN IMPORTED)
add_library(gtest_main UNKNOWN IMPORTED)
set_target_properties(gtest PROPERTIES
    IMPORTED_LOCATION ${binary_dir}/libgtest.a
)
set_target_properties(gtest_main PROPERTIES
    IMPORTED_LOCATION ${binary_dir}/libgtest_main.a
)
add_dependencies(gtest googletest)
add_dependencies(gtest_main googletest)

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
set_target_properties(${TEST_BINARY_NAME} PROPERTIES PRIVATE "-Wnofloat-equal")
target_include_directories(${TEST_BINARY_NAME} PRIVATE include)
target_include_directories(${TEST_BINARY_NAME} PRIVATE src)
target_include_directories(${TEST_BINARY_NAME} PRIVATE test)
target_link_libraries(${TEST_BINARY_NAME} gtest gtest_main pthread )
target_link_libraries(${TEST_BINARY_NAME} ${Boost_LIBRARIES} )
target_link_libraries(${TEST_BINARY_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

target_compile_options(${TEST_BINARY_NAME} PRIVATE "-Wno-unused-function")
