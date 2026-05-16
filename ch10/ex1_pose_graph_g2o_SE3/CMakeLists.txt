cmake_minimum_required(VERSION 3.10)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O2")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Eigen
Find_Package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} )


# g2o 
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3
        g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES}
        )

