
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(Threads REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
    ${PROJECT_SOURCE_DIR}/deps/geom/include
)

add_executable(nc_model model.cpp rs274_model.cpp ../Simulation.cpp ../Tool.cpp ../Stock.cpp ../rs274_base.cpp)
target_link_libraries(nc_model
    rs274ngc
    cxxcam
    geom
    ${CMAKE_THREAD_LIBS_INIT}
)
