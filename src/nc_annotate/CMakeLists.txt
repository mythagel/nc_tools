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
    ${PROJECT_SOURCE_DIR}/deps/clipper
    ${PROJECT_SOURCE_DIR}/deps/circle3d
)

add_executable(nc_annotate_curvature annotate_curvature.cpp rs274_annotate.cpp ../print_exception.cpp)
target_link_libraries(nc_annotate_curvature
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    nc_base
    circle3d
)

add_executable(nc_annotate_engagement annotate_engagement.cpp rs274_annotate.cpp ../print_exception.cpp)
target_link_libraries(nc_annotate_engagement
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    nc_base
    geom
    circle3d
    polyclipping
)
