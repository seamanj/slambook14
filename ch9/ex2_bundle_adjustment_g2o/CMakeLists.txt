cmake_minimum_required(VERSION 3.10)

project(bundle_adjustment_g2o)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})  
# target_link_libraries(bundle_adjustment_g2o Eigen3::Eigen) will only work on bundle_adjustment_g2o. Here we need Eigen on bal_coommon.cpp as well, so we include it globally.
add_library(bal_common common.cpp)

add_executable(bundle_adjustment_g2o
    bundle_adjustment_g2o.cpp
)

target_link_libraries(bundle_adjustment_g2o
    bal_common
    g2o_core
    g2o_stuff
    g2o_solver_csparse
    
)