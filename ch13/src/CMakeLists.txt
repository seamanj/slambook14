add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})

# 不需要再单独设置，会继承最外层的 CMAKE_RUNTIME_OUTPUT_DIRECTORY
# 如果希望显式设置，可以这样：
set_target_properties(myslam PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
)