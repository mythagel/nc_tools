
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(SFML 2 COMPONENTS system window graphics audio REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(OpenSceneGraph 3.2.0 REQUIRED osgViewer osgGA)
FIND_PACKAGE(Threads REQUIRED)

include_directories(
    ${SFML_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
    ${PROJECT_SOURCE_DIR}/deps/geom/include
)

add_executable(nc_model model.cpp rs274_model.cpp Simulation.cpp)
target_link_libraries(nc_model
    ${SFML_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    rs274ngc
    cxxcam
    geom
    ${CMAKE_THREAD_LIBS_INIT}
)
