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


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})




# octomap 
#find_package(octomap REQUIRED)


# 打印所有以 OCTOMAP 开头的变量
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     if(_variableName MATCHES "octomap|OCTOMAP|OCTOMATH")
#         message(STATUS "${_variableName}=${${_variableName}}")
#     endif()
# endforeach()

# OctoMap - 手动设置路径
set(Octomap_ROOT "D:/Software/msys64/ucrt64")
include_directories(${Octomap_ROOT}/include)  # 这已经包含了 octomap 头文件路径

add_executable(octomap_mapping octomap_mapping.cpp)

target_link_libraries(octomap_mapping 
    ${OpenCV_LIBS}
    # ${PCL_LIBS}
    octomap
    octomath
)