
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(Lua REQUIRED)

include_directories(
    ${LUA_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
)

add_library(nc_base STATIC rs274_base.cpp)
target_link_libraries(nc_base
    ${LUA_LIBRARIES}
    nc_lua
    rs274ngc
    cxxcam
)
