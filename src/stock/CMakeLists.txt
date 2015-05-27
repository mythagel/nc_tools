
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

include_directories(
    ${PROJECT_SOURCE_DIR}/deps/geom/include
)

add_executable(nc_stock stock.cpp)
target_link_libraries(nc_stock
    geom
)
