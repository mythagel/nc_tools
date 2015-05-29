
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
FIND_PACKAGE(Lua REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/geom/include
)

add_executable(nc_stock stock.cpp ../print_exception.cpp)
target_link_libraries(nc_stock
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    geom
)
