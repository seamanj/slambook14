cmake_minimum_required(VERSION 3.10)
project(pointcloud_mapping)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_definitions(-DPCL_SILENCE_MALLOC_WARNING=1)
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)

# opencv 
find_package(OpenCV REQUIRED)

# eigen 
find_package(Eigen3 REQUIRED)

# 手动设置 PCL 路径
set(PCL_ROOT "D:/Software/msys64/ucrt64")
set(PCL_INCLUDE_DIR "${PCL_ROOT}/include/pcl-1.15")
set(PCL_LIB_DIR "${PCL_ROOT}/lib")

# 包含目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIR})
include_directories(${PCL_ROOT}/include)

# 手动链接库
file(GLOB PCL_LIBS "${PCL_LIB_DIR}/libpcl_*.dll.a")
message("Found PCL libraries: ${PCL_LIBS}")

add_executable(surfel_mapping surfel_mapping.cpp)
target_link_libraries(surfel_mapping 
    ${OpenCV_LIBS}
    ${PCL_LIBS}
    pthread
)