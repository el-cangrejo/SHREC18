cmake_minimum_required(VERSION 2.8)

project( SHREC18 )

set(LIBIGL_DIR "/home/dimitrisd/Documents/Development/libigl/shared")
find_package(PCL 1.7 REQUIRED)
#find_package(LIBIGL REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
file(GLOB SOURCES "src/*.cpp")

add_definitions(${PCL_DEFINITIONS})
add_executable(SHREC18 ${SOURCES} "./src/main.cpp" "./src/comparison.h")

target_link_libraries(SHREC18 ${PCL_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -std=c++11")
