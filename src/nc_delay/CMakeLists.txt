
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(Threads REQUIRED)
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
FIND_PACKAGE(Lua REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/deps/rs274ngc/include
    ${PROJECT_SOURCE_DIR}/deps/cxxcam/include
)

add_executable(nc_delay delay.cpp rs274_delay.cpp ../print_exception.cpp)
target_link_libraries(nc_delay
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${LUA_LIBRARIES}
    rs274ngc
    nc_base
)
