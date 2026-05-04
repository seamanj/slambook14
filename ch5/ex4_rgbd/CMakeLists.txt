cmake_minimum_required(VERSION 3.10)
project(rgbd)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Threads REQUIRED)
find_package(Pangolin REQUIRED)

set(OpenCV_DIR "D:\\Software\\msys64\\ucrt64\\lib\\cmake\\opencv4")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Sophus::Sophus) 