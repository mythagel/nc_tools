FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
FIND_PACKAGE(Lua REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED NO_MODULE)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/geom/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
)

add_executable(nc_transform transform.cpp rs274_transform.cpp ../print_exception.cpp)
target_link_libraries(nc_transform
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    Eigen3::Eigen
    rs274ngc
    nc_base
    geom
    cxxcam
)
