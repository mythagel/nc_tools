FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED NO_MODULE)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/svg_path/include
)

add_executable(nc_svgpath svgpath.cpp ../print_exception.cpp)
target_link_libraries(nc_svgpath
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    svg_path
)
