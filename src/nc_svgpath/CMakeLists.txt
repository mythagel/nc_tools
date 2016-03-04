
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
)

add_executable(nc_svgpath svgpath.cpp ../print_exception.cpp)
target_link_libraries(nc_svgpath
    ${Boost_LIBRARIES}
    svg_path
)
