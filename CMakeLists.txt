cmake_minimum_required(VERSION 2.8.8)

# ---[ Solution name
project(TSDF C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fpermissive")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tsdf tsdf.cpp marching_cubes.cpp marching_cubes_table.h marching_cubes.h)
add_executable(simulatetsdf simulatetsdf.cpp)
target_link_libraries(tsdf ${PCL_LIBRARIES})
target_link_libraries(simulatetsdf ${PCL_LIBRARIES})

install(TARGETS tsdf simulatetsdf RUNTIME DESTINATION bin COMPONENT main)
install(FILES ${srcs} DESTINATION app COMPONENT main)
