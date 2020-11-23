project (surfer VERSION "${SURFVERSION}" DESCRIPTION "surfer")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package( Boost REQUIRED iostreams )
if ( NOT Boost_FOUND )
  message(STATUS "Boost not found.")
  return()
endif()
include_directories(${Boost_INCLUDE_DIRS})

add_executable(surfer
                       main.cpp )


if ( MSVC )
  target_sources(surfer
                       PRIVATE ../surf/getopt-for-windows/getopt.c )
  target_include_directories(surfer PRIVATE ../surf/getopt-for-windows )
endif()

target_link_libraries(surfer surflib)
target_link_libraries(surfer ${Boost_LIBRARIES} )

target_include_directories(surfer PRIVATE ../surf/include)
target_include_directories(surfer PRIVATE ../surf/src)

#### TESTING ##
if(TEST_SUITE)
  set( TEST_BINARY_NAME runtests_cc )
  include(GoogleTest)
  add_executable( ${TEST_BINARY_NAME}
    test/RunTestData.cpp
    main.cpp
    )
  gtest_add_tests(TARGET ${TEST_BINARY_NAME})
  set_target_properties(${TEST_BINARY_NAME} PROPERTIES PRIVATE "-Wnofloat-equal")
  target_compile_definitions(${TEST_BINARY_NAME} PRIVATE -DCMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}" )
  target_compile_definitions(${TEST_BINARY_NAME} PRIVATE -DSURF_TEST_SUITE=1 )

  target_include_directories(${TEST_BINARY_NAME} PRIVATE ../surf/include)
  target_include_directories(${TEST_BINARY_NAME} PRIVATE ../surf/src)
  target_include_directories(${TEST_BINARY_NAME} PRIVATE test)
  target_link_libraries(${TEST_BINARY_NAME} gtest gtest_main pthread )
  target_link_libraries(${TEST_BINARY_NAME} surflib )
  target_link_libraries(${TEST_BINARY_NAME} stdc++fs )
  target_link_libraries(${TEST_BINARY_NAME} ${Boost_LIBRARIES} )

  target_compile_options(${TEST_BINARY_NAME} PRIVATE "-Wno-unused-function")
ENDIF()
