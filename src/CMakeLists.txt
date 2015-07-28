
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
add_subdirectory(nc_lua)
add_subdirectory(nc_base)

add_subdirectory(backplot)
add_subdirectory(model)
add_subdirectory(stock)
add_subdirectory(delay)
