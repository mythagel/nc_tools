
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
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
    ${PROJECT_SOURCE_DIR}/deps/geom/include
    ${PROJECT_SOURCE_DIR}/deps/clipper
)

add_executable(nc_mill_roughing mill_roughing.cpp ../print_exception.cpp)
target_link_libraries(nc_mill_roughing
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    nc_base
    geom
    polyclipping
)
