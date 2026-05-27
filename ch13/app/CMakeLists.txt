add_executable(run_kitti_stereo run_kitti_stereo.cpp)
target_link_libraries(run_kitti_stereo myslam ${THIRD_PARTY_LIBS})

# 可选：显式设置可执行文件输出路径
set_target_properties(run_kitti_stereo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
