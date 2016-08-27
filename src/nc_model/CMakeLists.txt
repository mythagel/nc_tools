
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(Threads REQUIRED)
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
FIND_PACKAGE(Lua REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
    ${PROJECT_SOURCE_DIR}/deps/geom/include
    ${PROJECT_SOURCE_DIR}/deps/cork/include
    ${PROJECT_SOURCE_DIR}/deps/cork/src/
    ${PROJECT_SOURCE_DIR}/deps/cork/src/util
    ${PROJECT_SOURCE_DIR}/deps/cork/src/file_formats
    ${PROJECT_SOURCE_DIR}/deps/cork/src/math
    ${PROJECT_SOURCE_DIR}/deps/cork/src/isct
    ${PROJECT_SOURCE_DIR}/deps/cork/src/accel
)

add_executable(nc_model model.cpp rs274_model.cpp ../Simulation.cpp ../Tool.cpp ../Stock.cpp ../print_exception.cpp)
target_link_libraries(nc_model
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    cxxcam
    geom
    cork
    nc_base
)
