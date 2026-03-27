FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)

# Boost uBLAS
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/svg_path/include
)

add_executable(nc_svgpath svgpath.cpp ../print_exception.cpp)
target_link_libraries(nc_svgpath
    ${Boost_LIBRARIES}
    svg_path
)
