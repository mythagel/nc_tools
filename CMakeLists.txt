CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(nc_tools)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

ENABLE_TESTING()
IF(NOT TARGET check)
	ADD_CUSTOM_TARGET(check COMMAND ${CMAKE_CTEST_COMMAND})
ENDIF()

SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/" )

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

ADD_SUBDIRECTORY(deps/rs274ngc)
ADD_SUBDIRECTORY(deps/cxxcam)
ADD_SUBDIRECTORY(deps/svg_path)
ADD_SUBDIRECTORY(deps/geom)
ADD_SUBDIRECTORY(deps/clipper)
ADD_SUBDIRECTORY(deps/circle3d)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test)
