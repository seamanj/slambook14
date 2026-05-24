cmake_minimum_required( VERSION 3.10
 )
project( loop_closure )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# fbow
find_package(fbow REQUIRED)
include_directories(${fbow_INCLUDE_DIRS})

add_executable(loop_closure loop_closure.cpp)
target_link_libraries(loop_closure 
    ${OpenCV_LIBS} 
    ${fbow_LIBS}
)