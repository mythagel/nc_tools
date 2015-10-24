
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(Lua REQUIRED)

include_directories(
    ${LUA_INCLUDE_DIR}
)

add_library(nc_lua STATIC luaconfig.cpp state.cpp)
target_link_libraries(nc_lua
    ${LUA_LIBRARIES}
)
