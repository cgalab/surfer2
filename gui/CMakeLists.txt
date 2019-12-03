project (surfgui VERSION "${SURFVERSION}" DESCRIPTION "surfgui")

find_package(Qt5Widgets REQUIRED)
find_package(CGAL COMPONENTS Qt5 Core REQUIRED)
include(${CGAL_USE_FILE})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

add_executable(surfgui
                       main.cpp
                       mainwindow.cpp
                       InputGraphicsItem.cpp
                       KineticTriangulationGraphicsItem.cpp
                       )
target_link_libraries(surfgui ${QT_LIBRARIES})
target_link_libraries(surfgui ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
TARGET_LINK_LIBRARIES( surfgui
                       surflib )
target_include_directories(surfgui PRIVATE ../surf/include)
target_include_directories(surfgui PRIVATE ../surf/src)

qt5_use_modules(surfgui Widgets)
