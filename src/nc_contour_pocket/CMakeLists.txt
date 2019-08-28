
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
FIND_PACKAGE(Lua REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
    ${PROJECT_SOURCE_DIR}/deps/clipper
)

add_executable(nc_contour_offset contour_offset.cpp rs274_clipper_path.cpp ../print_exception.cpp)
target_link_libraries(nc_contour_offset
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    cxxcam
    nc_base
    polyclipping
)

add_executable(nc_contour_pocket contour_pocket.cpp rs274_clipper_path.cpp ../print_exception.cpp)
target_link_libraries(nc_contour_pocket
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    cxxcam
    nc_base
    polyclipping
)

add_executable(nc_spiral_pocket spiral_pocket.cpp rs274_clipper_path.cpp ../print_exception.cpp)
target_link_libraries(nc_spiral_pocket
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    cxxcam
    nc_base
    polyclipping
)

add_executable(nc_contour_spiral contour_spiral.cpp rs274_clipper_path.cpp ../print_exception.cpp)
target_link_libraries(nc_contour_spiral
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    cxxcam
    nc_base
    polyclipping
)

add_executable(nc_contour_drill contour_drill.cpp rs274_clipper_path.cpp ../print_exception.cpp)
target_link_libraries(nc_contour_drill
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    cxxcam
    nc_base
    polyclipping
)

add_executable(nc_contour_profile contour_profile.cpp rs274_clipper_path.cpp ../print_exception.cpp)
target_link_libraries(nc_contour_profile
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    cxxcam
    nc_base
    polyclipping
)
