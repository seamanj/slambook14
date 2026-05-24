cmake_minimum_required( VERSION 3.10
 )
project( feature_training )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# DBoW3
set(DBoW3_DIR "D:/Software/msys64/ucrt64/cmake/DBoW3")
find_package(DBoW3 REQUIRED)
include_directories(${DBoW3_INCLUDE_DIRS})

add_executable(feature_training feature_training.cpp)
target_link_libraries(feature_training 
    ${OpenCV_LIBS} 
    ${DBoW3_LIBS}
)