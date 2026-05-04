cmake_minimum_required(VERSION 3.10)
project(stereoVision)


find_package(Threads REQUIRED)
find_package(Pangolin REQUIRED)

set(OpenCV_DIR "D:\\Software\\msys64\\ucrt64\\lib\\cmake\\opencv4")
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})