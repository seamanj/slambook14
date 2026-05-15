cmake_minimum_required(VERSION 3.10)

project(bundle_adjustment_ceres)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR} )


add_library(bal_common common.cpp)
add_executable(bundle_adjustment_ceres bundle_adjustment_ceres.cpp)

target_link_libraries(bundle_adjustment_ceres ${CERES_LIBRARIES} bal_common)

