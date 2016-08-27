CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
PROJECT(cork)

SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" )

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
IF(UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

FIND_PACKAGE(GMP REQUIRED)

# TODO find GMP
INCLUDE_DIRECTORIES(
    include
    src/util
    src/file_formats
    src/math
    src/isct
    src/accel
    ${GMP_INCLUDE_DIR}
)

ADD_DEFINITIONS()
SET_PROPERTY(
    SOURCE src/isct/triangle.c
    APPEND
    PROPERTY COMPILE_DEFINITIONS
    NO_TIMER REDUCED CDT_ONLY TRILIBRARY ANSI_DECLARATORS
)
ADD_LIBRARY(triangle STATIC src/isct/triangle.c)

ADD_LIBRARY(cork STATIC
    src/cork.cpp
    src/util/timer.cpp
    src/util/log.cpp
    src/isct/empty3d.cpp
    src/isct/quantization.cpp
    src/file_formats/files.cpp
    src/file_formats/ifs.cpp
    src/file_formats/off.cpp
)
TARGET_LINK_LIBRARIES(cork triangle ${GMP_LIBRARIES})

ADD_EXECUTABLE(cork_cli src/main.cpp)
TARGET_LINK_LIBRARIES(cork_cli cork)
