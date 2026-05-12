# 依赖库版本说明

本项目使用的第三方库版本如下（截止到：2026年5月8日）。

---

## 核心依赖

- **Eigen**：3.4.0  
- **Ceres Solver**：2.3.0  
- **Sophus**：1.24.6  
- **g2o**：1.0.0  

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