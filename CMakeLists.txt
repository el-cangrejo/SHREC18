cmake_minimum_required(VERSION 2.8)

project( SHREC18 )

find_package(PCL 1.7 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")

add_definitions(${PCL_DEFINITIONS})
add_executable(SHREC18 ${SOURCES})

target_link_libraries(SHREC18 ${PCL_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
