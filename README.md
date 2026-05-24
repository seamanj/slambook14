# 依赖库版本说明

本项目使用的第三方库版本如下（截止到：2026年5月8日）。

---

## 核心依赖

- **Eigen**：3.4.0  
- **Ceres Solver**：2.2.0  
- **Sophus**：1.24.6  
- **g2o**：1.0.0  
- **gtsam**：4.3.0  
- **fdow**: 0.0.1
- **PCL**: 1.15.1 
- **Octomap**: 1.10.0
---

## 说明

- **Eigen**  
  用于矩阵运算与线性代数计算，是整个优化与几何计算的基础库。

  由于我们用的MINGW, 记得加上
  
  ```cmake
  if(MSVC OR (MINGW AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
    # 对于 MinGW/GCC，添加更好的调试符号和内联优化
    # add_compile_options(-O2 -g -finline-functions -fno-inline-small-functions)
    add_compile_options(-O2)
    # 如果仍有问题，尝试降低优化级别
    # add_compile_options(-O1)
  endif()
  ```

- **Ceres Solver**  
  用于非线性优化（如 PnP、BA、ICP 等问题）。
  make的时候关掉CUDA
  ```cmake
  cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=D:/Software/msys64/ucrt64 -DBUILD_TESTING=OFF -DUSE_CUDA=OFF
  ```

- **Sophus**  
  用于李群表示（SE(3)、SO(3)），便于位姿更新与指数映射。


- **g2o**  
  用于图优化（Graph Optimization），主要用于 BA 等结构化优化问题。

  为了应用g2o::LinearSolverCSparse, 我们需要在G2O里面支持CSPARSE. 具体用法见ch9_ex2

  1. 先安装suitesparse这个库
  ```
  pacman -S mingw-w64-ucrt-x86_64-suitesparse
  ```
  2. 再编译G2O带上`-DG2O_USE_CSPARSE=ON`
  ```cmake
  cmake .. -G "MinGW Makefiles" -DCMAKE_INSTALL_PREFIX=/ucrt64 -DCMAKE_BUILD_TYPE=Release -DG2O_BUILD_APPS=ON -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_CSPARSE=ON -DCSPARSE_INCLUDE_DIR=/ucrt64/include/suitesparse -DCSPARSE_LIBRARY=/ucrt64/lib/libcxsparse.dll.a -DCMAKE_CXX_FLAGS="-I/ucrt64/include/suitesparse" -DCMAKE_C_FLAGS="-I/ucrt64/include/suitesparse"

  ```

  3. 如果需要g2o_viewer, 我们需要先编译这个libQGLViewer这个库
  ```
  git clone https://github.com/GillesDebunne/libQGLViewer.git
  cd libQGLViewer
  # 1. 先编译核心库 QGLViewer
  cd /d/Software/libQGLViewer/QGLViewer

  # 2. 清理之前的编译残留（如果有）
  make clean

  # 3. 生成 Makefile 并编译核心库
  qmake PREFIX=/ucrt64
  make -j32

  # 4. 手动安装
  # QGLViewer没有make install, 需要自己手动复制
  # 查看当前目录下生成的库文件
  ls -la *.dll *.a 2>/dev/null

  # 查看是否已经安装到 /ucrt64 目录
  ls -la /ucrt64/lib/libQGLViewer* 2>/dev/null
  ls -la /ucrt64/bin/libQGLViewer* 2>/dev/null

  # 如果库文件只存在于当前目录而没有安装到系统目录，需要手动复制：
  # 手动复制库文件到系统目录
  cp -v libQGLViewer3.dll /ucrt64/bin/
  cp -v libQGLViewer3.a /ucrt64/lib/
  cp -v libQGLViewerd3.dll /ucrt64/bin/
  cp -v libQGLViewerd3.a /ucrt64/lib/

  # 复制头文件
  cp -rv ../QGLViewer /ucrt64/include/

  # 确认文件已复制成功
  ls -la /ucrt64/bin/libQGLViewer*.dll
  ls -la /ucrt64/lib/libQGLViewer*.a
  ls -la /ucrt64/include/QGLViewer/
  ```

  然后我们可以编译一下它自带的simpleViewer, 如果发现它用的QGLViewer2, 手动改为QGLViewer3

  ```
  cd /d/Software/libQGLViewer/examples
  cp simpleViewer.pro simpleViewer.pro.bak

  sed -i 's/QGLViewer2/QGLViewer3/g' examples.pri

  # 返回重新编译
  cd simpleViewer
  make clean
  qmake
  make -j32
  # 生成D:\Software\libQGLViewer\examples\simpleViewer\release\simpleViewer.exe
  ```
  界面长这样:
  ![simpleViewer](./resource/simpleViewer.png)

  4. 再编译G2O, `-DG2O_BUILD_APPS=ON`
  ```cmake
  cmake .. -G "MinGW Makefiles" -DCMAKE_INSTALL_PREFIX=/ucrt64 -DCMAKE_BUILD_TYPE=Release -DG2O_BUILD_APPS=ON -DG2O_BUILD_EXAMPLES=OFF -DG2O_USE_CSPARSE=ON -DCSPARSE_INCLUDE_DIR=/ucrt64/include/suitesparse -DCSPARSE_LIBRARY=/ucrt64/lib/libcxsparse.dll.a -DQGLVIEWER_INCLUDE_DIR=/ucrt64/include -DQGLVIEWER_LIBRARY=/ucrt64/lib/libQGLViewer3.a -DCMAKE_CXX_FLAGS="-I/ucrt64/include/suitesparse" -DCMAKE_C_FLAGS="-I/ucrt64/include/suitesparse"
  ```

  打开g2o_viewer, 让我们加载第10章的位姿图文件, 界面长这样:
  ![g2o_viewer](./resource/g2o_viewer.png)


  如果cmake找不到QGLViewer, 我们尝试下自己写个cmake配置
  ```
  # 1. 创建 QGLViewer 的 CMake 配置文件目录
  mkdir -p /ucrt64/lib/cmake/QGLViewer

  # 2. 创建配置文件
  cat > /ucrt64/lib/cmake/QGLViewer/QGLViewerConfig.cmake << 'EOF'
  # QGLViewer configuration for Windows/MSYS2 UCRT64
  set(QGLVIEWER_FOUND TRUE)
  set(QGLVIEWER_INCLUDE_DIRS /ucrt64/include)
  set(QGLVIEWER_LIBRARIES /ucrt64/lib/libQGLViewer3.a)
  set(QGLVIEWER_LIBRARY ${QGLVIEWER_LIBRARIES})
  set(QGLVIEWER_INCLUDE_DIR ${QGLVIEWER_INCLUDE_DIRS})

  # Create imported target
  add_library(QGLViewer::QGLViewer UNKNOWN IMPORTED)
  set_target_properties(QGLViewer::QGLViewer PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${QGLVIEWER_INCLUDE_DIRS}"
      IMPORTED_LOCATION "${QGLVIEWER_LIBRARIES}"
  )

  # Also create non-namespaced target for compatibility
  add_library(QGLViewer UNKNOWN IMPORTED)
  set_target_properties(QGLViewer PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${QGLVIEWER_INCLUDE_DIRS}"
      IMPORTED_LOCATION "${QGLVIEWER_LIBRARIES}"
  )
  EOF
  ```



- **gstam**  
用于基于因子图的概率建模与非线性优化，支持 SLAM、三维重建与位姿图优化等问题的增量式求解。

  gstam对MinGW支持比较差, 在Windows上默认支持MSVC, 所以我们需要手动改下:

  1. 打开`gstam\gstam\CMakeLists.txt`
  将
  ```
  set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "/w")
  ```
  改成
  ```
  if(WIN32)
    if(MSVC)
      set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "/w")
    else()
      set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-w")
    endif()
  else()
  ```

  注释掉    `#constrained`

  2. 打开`gstam\cmake\GtsamBuildTypes.cmake`
  注释掉
  `#-Werror                                        # Enable warnings as errors`
  3. 打开 `gtsam\gtsam\3rdparty\cephes\CMakeLists.txt`
  类似修改WIN32部分
  ```
  if(WIN32)
    if(MSVC)
      set_target_properties(cephes-gtsam PROPERTIES COMPILE_FLAGS /w)
    else()
      set_target_properties(cephes-gtsam PROPERTIES COMPILE_FLAGS -w)
    endif()
  endif()
  ```
  4. 打开`gtsam\cmake\dllexport.h.in`
  将WIN32部分改成
  ```
  #ifdef _WIN32
  #  ifndef GTSAM_SHARED_LIB
  #    define @library_name@_EXPORT
  #    define @library_name@_EXTERN_EXPORT extern
  #  else
  #    ifdef @library_name@_EXPORTS
  #      ifdef GTSAM_MINGW
          // MinGW 使用 GCC 可见性属性，而不是 __declspec
  #        define @library_name@_EXPORT __attribute__((visibility("default")))
  #        define @library_name@_EXTERN_EXPORT __attribute__((visibility("default"))) extern
  #      else
          // MSVC 使用 __declspec
  #        define @library_name@_EXPORT __declspec(dllexport)
  #        define @library_name@_EXTERN_EXPORT __declspec(dllexport) extern
  #      endif
  #    else
  #      ifdef GTSAM_MINGW
          // MinGW 导入时不需要特殊标记
  #        define @library_name@_EXPORT
  #        define @library_name@_EXTERN_EXPORT extern
  #      else
  #        define @library_name@_EXPORT __declspec(dllimport)
  #        define @library_name@_EXTERN_EXPORT __declspec(dllimport)
  #      endif
  #    endif
  #  endif
  #else
  ```
  5. 最后编译 

  ```
  cmake .. \
    -G "MinGW Makefiles" \
    -DCMAKE_INSTALL_PREFIX=/ucrt64 \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_SUPPORT_NESTED_DISSECTION=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_CXX_FLAGS="-D_USE_MATH_DEFINES -fpermissive"
  ```

- **DBow3** 
  DBoW3 是 DBow2 库的改进版本，这是一个开源的 C++ 库，主要用于将图像索引并转换为词袋表示。它通过实现分层树结构，在图像特征空间中进行近似最近邻搜索，从而创建视觉词汇表。此外，DBoW3 还实现了一个带有倒排文件和直接文件的图像数据库，用于对图像进行索引，支持快速查询和特征比较。

  在 Windows 上，LIB_INSTALL_DIR 变量没有被正确设置。我们需要修改下CMakeLists.txt
  ```cmake
  if(WIN32)
    # Postfix of DLLs:
    SET(PROJECT_DLLVERSION "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}")
    SET(RUNTIME_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls and binaries")
    SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for binaries")
    SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Directory for dlls")
    # 添加这一行：为 Windows 设置 LIB_INSTALL_DIR
    set(LIB_INSTALL_DIR "cmake" CACHE STRING "Install location of CMake config files")
  else()
    # Postfix of so's:
    set(PROJECT_DLLVERSION)
    set(LIB_INSTALL_DIR lib CACHE STRING "Install location of libraries (e.g. lib32 or lib64 for multilib installations)")
    SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/cmake/ /usr/${LIB_INSTALL_DIR}/cmake )
  endif()
  ```
  后面两行改为
  ```
  #INSTALL(FILES "${PROJECT_BINARY_DIR}/Find${PROJECT_NAME}.cmake" DESTINATION ${LIB_INSTALL_DIR}/cmake/ )
  INSTALL(FILES "${PROJECT_BINARY_DIR}/Find${PROJECT_NAME}.cmake" DESTINATION ${LIB_INSTALL_DIR} )
  #INSTALL(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME} )
  INSTALL(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" DESTINATION ${LIB_INSTALL_DIR}/${PROJECT_NAME} )
  ```

  在CMakeLists.txt文件里面
  ```
  # DBoW3
  set(DBoW3_DIR "D:/Software/msys64/ucrt64/cmake/DBoW3")
  find_package(DBoW3 REQUIRED)
  include_directories(${DBoW3_INCLUDE_DIRS})

  target_link_libraries(XXXXX 
      ${DBoW3_LIBS}
  )
  ```

- **fdow** 
  FBOW（Fast Bag of Words，快速词袋模型）是 DBow2/DBow3 库的一个高度优化版本。该库利用 AVX、SSE 和 MMX 指令集进行深度优化，显著提升了词袋向量的生成速度。在加载词汇表时，fbow 比 DBOW2 快约 80 倍（参见 tests 目录并自行测试）。在支持 AVX 指令集的机器上将图像转换为词袋向量时，其速度约为 DBOW2 的 6.4 倍。

  在mingw上使用, 需要修改一处地方. 打开`D:/Software/fbow/src/cpu.h`

  将
  ```
  #   if _WIN32
  #include <Windows.h>
  #include <intrin.h>
  #   elif defined(__GNUC__) || defined(__clang__)
  #include <cpuid.h>
  #define _XCR_XFEATURE_ENABLED_MASK  0
  #   else
  #       error "No cpuid intrinsic defined for compiler."
  #   endif
  ```
  替换成
  ```
  #   if _WIN32
  #include <Windows.h>
  #include <intrin.h>
  // 对于 MinGW，定义缺失的宏
  #ifndef _XCR_XFEATURE_ENABLED_MASK
  #define _XCR_XFEATURE_ENABLED_MASK 0
  #endif
  #   elif defined(__GNUC__) || defined(__clang__)
  #include <cpuid.h>
  #ifndef _XCR_XFEATURE_ENABLED_MASK
  #define _XCR_XFEATURE_ENABLED_MASK 0
  #endif
  #   else
  #       error "No cpuid intrinsic defined for compiler."
  #   endif
  ```

- **PCL** 
  The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing.

    ```cmake
    cmake .. -G "MinGW Makefiles" \
        -DCMAKE_INSTALL_PREFIX=/ucrt64 \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_visualization=OFF \
        -DBUILD_apps=OFF \
        -DBUILD_examples=OFF \
        -DBUILD_tools=OFF \
        -DBUILD_surface=ON
    ```


- **Octomap** 
  Octomap 是一个基于八叉树（Octree）的 3D 占据网格建图库，支持概率更新、多分辨率表示，常用于机器人导航与三维环境建模。


  如何不需要编译OCTOVIS
  ```cmake
  cmake .. -G "MinGW Makefiles" \
      -DCMAKE_INSTALL_PREFIX=/ucrt64 \
      -DBUILD_OCTOVIS_SUBPROJECT=OFF 
  ```

  如何需要编译OCTOVIS
  cmake .. -G "MinGW Makefiles" \
    -DCMAKE_INSTALL_PREFIX=/ucrt64 \
    -DOCTOVIS_USE_QGLVIEWER=installed \
    -DQGLViewer_INCLUDE_DIR=/ucrt64/include \
    -DQGLViewer_LIBRARY=/ucrt64/lib/libQGLViewer3.a \
    -DCMAKE_CXX_FLAGS="-I/ucrt64/include/QGLViewer"


  将D:\Software\octomap\octovis\CMakeLists.txt文件中
  ```
  set(octovis_SOURCES
      src/SceneObject.cpp src/PointcloudDrawer.cpp src/OcTreeDrawer.cpp
      src/SelectionBox.cpp src/TrajectoryDrawer.cpp src/ColorOcTreeDrawer.cpp
  )
  ```
  改成
  ```
  set(octovis_SOURCES
      src/SceneObject.cpp src/PointcloudDrawer.cpp src/OcTreeDrawer.cpp
      src/SelectionBox.cpp src/TrajectoryDrawer.cpp src/ColorOcTreeDrawer.cpp
      src/ViewerGui.cpp src/ViewerWidget.cpp src/ViewerSettings.cpp
      src/ViewerSettingsPanel.cpp src/ViewerSettingsPanelCamera.cpp src/CameraFollowMode.cpp
  )
  ```
---

## 兼容性说明

以上版本在当前工程环境中经过测试可正常编译运行。

如遇到编译或链接问题，请重点检查以下几点：

- 系统中是否存在多个版本的 Eigen（容易冲突）
- Ceres 与 g2o 是否使用了**同一版本 Eigen 编译**
- MSYS2 / MinGW 环境下是否混用了不同工具链（UCRT64 / MINGW64）
- 是否存在系统自带库与手动编译库冲突

---

## 构建环境

- Windows 11  
- MSYS2（UCRT64 或 MINGW64）  
- GCC / MinGW-w64 工具链  
- CMake ≥ 3.15  
- IDE：VS Code  

---

## 参考资料

Windows 上使用 MSYS2 + VSCode + MinGW 的配置方法可参考：

👉 https://www.bilibili.com/video/BV1L94y1N7e6