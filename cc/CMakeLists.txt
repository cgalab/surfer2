project (surfer VERSION "${SURFVERSION}" DESCRIPTION "surfer")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package( Boost REQUIRED iostreams )
if ( NOT Boost_FOUND )
  message(STATUS "Boost not found.")
  return()
endif()
include_directories(${Boost_INCLUDE_DIRS})


add_executable(surfer
                       main.cpp
                       )

target_link_libraries(surfer surflib)
target_link_libraries(surfer ${Boost_LIBRARIES} )

target_include_directories(surfer PRIVATE ../surf/include)
target_include_directories(surfer PRIVATE ../surf/src)
