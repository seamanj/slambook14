# 依赖库版本说明

本项目使用的第三方库版本如下（截止到：2026年5月8日）。

---

## 核心依赖

- **Eigen**：5.0.1  
- **Ceres Solver**：2.3.0  
- **Sophus**：1.24.6  
- **g2o**：1.0.0  

---

## 说明

- **Eigen**  
  用于矩阵运算与线性代数计算，是整个优化与几何计算的基础库。

- **Ceres Solver**  
  用于非线性优化（如 PnP、BA、ICP 等问题）。

- **Sophus**  
  用于李群表示（SE(3)、SO(3)），便于位姿更新与指数映射。

  ⚠️ 编译注意事项：  
  在编译 Sophus 时，建议去掉 Eigen 的版本限制，以避免版本冲突。

  在 `CMakeLists.txt` 中：

  ```cmake
  find_package(Eigen3 3.4.0 REQUIRED)
  ```
  修改为

  ```cmake
  find_package(Eigen3 REQUIRED)
  ```