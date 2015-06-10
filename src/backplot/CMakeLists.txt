
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(SFML 2 COMPONENTS system window graphics audio REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(OpenSceneGraph 3.2.0 REQUIRED osgViewer osgGA)
FIND_PACKAGE(Threads REQUIRED)
FIND_PACKAGE(Lua REQUIRED)

include_directories(
    ${SFML_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
)

add_executable(nc_backplot backplot.cpp rs274_backplot.cpp)
target_link_libraries(nc_backplot
    ${SFML_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${LUA_LIBRARIES}
    rs274ngc
    cxxcam
    nc_base
)
