cmake_minimum_required(VERSION 3.10)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O2")

# 手动设置 GTSAM 路径（因为你的 GTSAM 编译在特定位置）
# set(GTSAM_DIR "/ucrt64/lib/cmake/GTSAM")
# 或者
# set(GTSAM_DIR "/d/Software/gtsam/build")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Sophus 
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# GTSAM 
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIRS})
# set(GTSAM_LIBRARY "D:/Software/msys64/ucrt64/lib/libgtsam.dll.a")

add_executable(pose_graph_gtsam pose_graph_gtsam.cpp)

# 链接所有需要的库
target_link_libraries(pose_graph_gtsam
    Sophus::Sophus
    gtsam                   # 直接指定
)

# 如果需要，手动添加链接目录
# target_link_directories(pose_graph_gtsam PRIVATE /ucrt64/lib)