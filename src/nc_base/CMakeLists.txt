
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
)

add_library(nc_base STATIC rs274_base.cpp)
target_link_libraries(nc_base
    nc_lua
    rs274ngc
    cxxcam
)
