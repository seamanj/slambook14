#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <format>
// #include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <sophus/se3.hpp>
#include <g2o/core/robust_kernel_impl.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/manifold.h>

/*

sophus SE3 parameterization: [translation, rotation]
*/

using namespace std;

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// baseline
double baseline = 0.573;
// paths
string left_file = "../left.png";
string disparity_file = "../disparity.png";
// boost::format fmt_others("./%06d.png");    // other files
string fmt_others = "../{0:06d}.png"; // other files - C++20 format string
// useful typedefs
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/// class for accumulator jacobians in parallel
class JacobianAccumulator
{
public:
  JacobianAccumulator(
      const cv::Mat &img1_,
      const cv::Mat &img2_,
      const VecVector2d &px_ref_,
      const vector<double> depth_ref_,
      Sophus::SE3d &T21_) : img1(img1_), img2(img2_), px_ref(px_ref_), depth_ref(depth_ref_), T21(T21_)
  {
    projection = VecVector2d(px_ref.size(), Eigen::Vector2d(0, 0));
  }

  /// accumulate jacobians in a range
  void accumulate_jacobian(const cv::Range &range);

  /// get hessian matrix
  Matrix6d hessian() const { return H; }

  /// get bias
  Vector6d bias() const { return b; }

  /// get total cost
  double cost_func() const { return cost; }

  /// get projected points
  VecVector2d projected_points() const { return projection; }

  /// reset h, b, cost to zero
  void reset()
  {
    H = Matrix6d::Zero();
    b = Vector6d::Zero();
    cost = 0;
  }

private:
  const cv::Mat &img1;
  const cv::Mat &img2;
  const VecVector2d &px_ref;
  const vector<double> depth_ref;
  Sophus::SE3d &T21;
  VecVector2d projection; // projected points

  std::mutex hessian_mutex;
  Matrix6d H = Matrix6d::Zero();
  Vector6d b = Vector6d::Zero();
  double cost = 0;
};

/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21);

/**
 * pose estimation using direct method
 * @param img1
 * @param img2
 * @param px_ref
 * @param depth_ref
 * @param T21
 */
void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21);

void DirectPoseEstimationSingleLayerG2O(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21);

void DirectPoseEstimationSingleLayerCeres(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21);

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
  // boundary check
  if (x < 0)
    x = 0;
  if (y < 0)
    y = 0;
  if (x >= img.cols)
    x = img.cols - 1;
  if (y >= img.rows)
    y = img.rows - 1;
  uchar *data = &img.data[int(y) * img.step + int(x)];
  float xx = x - floor(x);
  float yy = y - floor(y);
  return float(
      (1 - xx) * (1 - yy) * data[0] +
      xx * (1 - yy) * data[1] +
      (1 - xx) * yy * data[img.step] +
      xx * yy * data[img.step + 1]);
}

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void setToOriginImpl() override
  {
    _estimate = Sophus::SE3d();
  }

  // SE3 左乘更新：T <- exp(δξ) * T
  virtual void oplusImpl(const double *update) override
  {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> dx(update);
    _estimate = Sophus::SE3d::exp(dx) * _estimate;
  }

  virtual bool read(std::istream &) override { return true; }
  virtual bool write(std::ostream &) const override { return true; }
};

/// g2o edge with 3x3 patch
class EdgeDirectPoseOnly : public g2o::BaseUnaryEdge<9, Eigen::Matrix<double, 9, 1>, VertexPose>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeDirectPoseOnly(
      const Eigen::Vector2d &uv_ref,
      const Eigen::Matrix3d &K,
      double depth,
      const cv::Mat &img1,
      const cv::Mat &img2)
      : uv_ref_(uv_ref),
        K_(K),
        img1_(img1),
        img2_(img2)
  {
    K_inv_ = K_.inverse();

    // 3D point in ref frame
    p_ref_ = depth * (K_inv_ * Eigen::Vector3d(uv_ref_[0], uv_ref_[1], 1.0));
  }

  // =========================
  // compute error (patch energy)
  // =========================
  void computeError() override
  {
    const VertexPose *v = static_cast<const VertexPose *>(_vertices[0]);
    Sophus::SE3d T = v->estimate();

    Eigen::Vector3d p_cur = T * p_ref_;

    if (p_cur[2] <= 0)
    {
      _error.setZero();
      setLevel(1);
      return;
    }

    Eigen::Vector3d uv = K_ * p_cur;
    double u = uv[0] / uv[2];
    double v_ = uv[1] / uv[2];

    if (u < 1 || u >= img2_.cols - 1 ||
        v_ < 1 || v_ >= img2_.rows - 1)
    {
      _error.setZero();
      setLevel(1);
      return;
    }

    int idx = 0;

    for (int x = -1; x <= 1; x++)
      for (int y = -1; y <= 1; y++)
      {
        double I_ref = GetPixelValue(img1_,
                                     uv_ref_[0] + x,
                                     uv_ref_[1] + y);

        double I_cur = GetPixelValue(img2_,
                                     u + x,
                                     v_ + y);

        _error[idx++] = I_ref - I_cur;
      }
  }

  // =========================
  // Jacobian
  // =========================
  virtual void linearizeOplus() override
  {
    const VertexPose *pose = static_cast<const VertexPose *>(_vertices[0]);
    Sophus::SE3d T = pose->estimate();

    Eigen::Vector3d p_cur = T * p_ref_;

    double X = p_cur[0];
    double Y = p_cur[1];
    double Z = p_cur[2];

    double fx = K_(0, 0);
    double fy = K_(1, 1);

    double invZ = 1.0 / Z;
    double invZ2 = invZ * invZ;

    // projection Jacobian (2x6)
    Eigen::Matrix<double, 2, 6> J_proj;

    J_proj << fx * invZ, 0, -fx * X * invZ2,
        -fx * X * Y * invZ2, fx + fx * X * X * invZ2, -fx * Y * invZ,

        0, fy * invZ, -fy * Y * invZ2,
        -fy - fy * Y * Y * invZ2, fy * X * Y * invZ2, fy * X * invZ;

    // image gradient (central difference at projected point)
    double u = fx * X / Z + K_(0, 2);
    double v = fy * Y / Z + K_(1, 2);

    Eigen::Matrix<double, 9, 2> J_img;

    int idx = 0;

    for (int x = -1; x <= 1; x++)
      for (int y = -1; y <= 1; y++)
      {
        double gx = 0.5 * (GetPixelValue(img2_, u + 1 + x, v + y) -
                           GetPixelValue(img2_, u - 1 + x, v + y));

        double gy = 0.5 * (GetPixelValue(img2_, u + x, v + 1 + y) -
                           GetPixelValue(img2_, u + x, v - 1 + y));

        J_img(idx, 0) = gx;
        J_img(idx, 1) = gy;
        idx++;
      }

    _jacobianOplusXi = -J_img * J_proj;
  }

  virtual bool read(std::istream &) override { return true; }
  virtual bool write(std::ostream &) const override { return true; }

private:
  Eigen::Vector2d uv_ref_;
  Eigen::Vector3d p_ref_;

  Eigen::Matrix3d K_, K_inv_;

  const cv::Mat &img1_;
  const cv::Mat &img2_;
};

/*
tj : 注意angle-axis+translation 和 Sophus SE3的参数化区别：
- angle-axis+translation: [tx, ty, tz, ax, ay, az]，其中ax, ay, az是旋转向量的分量，表示旋转轴乘以旋转角度。
- Sophus SE3: [tx, ty, tz, rx, ry, rz]，其中rx, ry, rz是旋转向量的分量，表示旋转轴乘以旋转角度（与angle-axis相同），但在优化过程中通常使用Sophus的SE3类来处理旋转和平移的更新。
1. angle-axis + translation

参数：
x=[r,t]∈R^6
更新方式：

r←r+δr
t←t+δt

其中：

r：Rodrigues / angle-axis 旋转参数
t：平移

特点：

本质是在参数空间直接加法
旋转不是严格李群更新
translation 与 rotation 相互独立
默认 δt 在世界坐标系

问题：

exp(a^)exp(b^) 不等于 exp((a+b)^)
因此：
r+δr
并不严格等价于：
exp(δr^)exp(r^)

大旋转时可能：

数值不稳定
收敛域较小
Hessian 不一致

2. SE(3) / Sophus / Lie Algebra 优化
位姿：
T∈SE(3)

增量：
δξ=[
δρ
δϕ]∈se(3)

更新：

T←exp(δξ∧)T

特点：

严格李群更新
始终保持 SE(3) 结构
rotation 与 translation 耦合
δt 定义在局部坐标系
数值稳定性更好

这是现代：

VO
SLAM
Bundle Adjustment

的标准做法。
*/

/*
1. 你现在“不是 manifold optimization”

这是你当前最大的理论问题。

你现在：
camera[6]
→ xi
→ T = exp(xi)
然后 ceres 优化：
xi += delta
这其实不是：
T←exp(δξ^)T
而是：
ξ←ξ+δξ
再：
T=exp(ξ^)

exp(a^)exp(b^)不等于exp((a+b)^)

所以：
你现在优化的是：
“李代数参数空间”
不是：
“SE3 manifold”

正确做法应该使用LocalParameterization或者Manifold

让 Ceres 做：
T←exp(δξ^)T
这是标准 SLAM 做法。

 */

class SE3Manifold : public ceres::Manifold
{
public:
  // dimension
  int AmbientSize() const override { return 6; }
  int TangentSize() const override { return 6; }

  // x_plus_delta = exp(delta) * x
  bool Plus(const double *x,
            const double *delta,
            double *x_plus_delta) const override
  {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> dx(delta);

    Sophus::SE3d T = Sophus::SE3d::exp(xi);
    Sophus::SE3d dT = Sophus::SE3d::exp(dx);

    Sophus::SE3d T_new = dT * T;

    Eigen::Map<Eigen::Matrix<double, 6, 1>> xi_new(x_plus_delta);
    xi_new = T_new.log();

    return true;
  }

  // inverse: y - x
  bool Minus(const double *y,
             const double *x,
             double *y_minus_x) const override
  {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi1(y);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi2(x);

    Sophus::SE3d T1 = Sophus::SE3d::exp(xi1);
    Sophus::SE3d T2 = Sophus::SE3d::exp(xi2);

    Sophus::SE3d dT = T2.inverse() * T1;

    Eigen::Map<Eigen::Matrix<double, 6, 1>> dx(y_minus_x);
    dx = dT.log();

    return true;
  }

  bool PlusJacobian(const double *x,
                    double *jacobian) const override
  {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
    return true;
  }

  bool MinusJacobian(const double *x,
                     double *jacobian) const override
  {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
    return true;
  }
};

class PhotometricErrorAnalytic : public ceres::SizedCostFunction<9, 6>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PhotometricErrorAnalytic(
      const Eigen::Vector2d &uv1,
      double d1,
      const Eigen::Matrix3d &K,
      const cv::Mat &img1,
      const cv::Mat &img2)
      : uv1_(uv1), d1_(d1), K_(K), img1_(img1), img2_(img2)
  {
    K_inv_ = K_.inverse();
  }

  virtual bool Evaluate(
      double const *const *parameters,
      double *residuals,
      double **jacobians) const override
  {
    // ---------------------------
    // SE3 (Lie algebra)
    // ---------------------------
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> xi(parameters[0]);
    Sophus::SE3d T = Sophus::SE3d::exp(xi);

    // ---------------------------
    // back project
    // ---------------------------
    Eigen::Vector3d p1 =
        d1_ * (K_inv_ * Eigen::Vector3d(uv1_[0], uv1_[1], 1.0));

    Eigen::Vector3d p2 = T * p1;

    double X = p2[0];
    double Y = p2[1];
    double Z = p2[2];

    if (Z <= 1e-6)
    {
      for (int i = 0; i < 9; i++)
        residuals[i] = 0;
      if (jacobians && jacobians[0])
        memset(jacobians[0], 0, sizeof(double) * 9 * 6);
      return true;
    }

    // ---------------------------
    // projection
    // ---------------------------
    double fx = K_(0, 0), fy = K_(1, 1);
    double u = fx * X / Z + K_(0, 2);
    double v = fy * Y / Z + K_(1, 2);

    if (u < 1 || u >= img2_.cols - 1 ||
        v < 1 || v >= img2_.rows - 1)
    {
      for (int i = 0; i < 9; i++)
        residuals[i] = 0;
      if (jacobians && jacobians[0])
        memset(jacobians[0], 0, sizeof(double) * 9 * 6);
      return true;
    }

    // ---------------------------
    // residual (patch 3x3)
    // ---------------------------
    int idx = 0;
    for (int x = -1; x <= 1; x++)
    {
      for (int y = -1; y <= 1; y++)
      {
        double ref = GetPixelValue(img1_, uv1_[0] + x, uv1_[1] + y);
        double cur = GetPixelValue(img2_, u + x, v + y);
        residuals[idx++] = ref - cur;
      }
    }

    // ---------------------------
    // Jacobian
    // ---------------------------
    if (jacobians && jacobians[0])
    {
      Eigen::Map<Eigen::Matrix<double, 9, 6, Eigen::RowMajor>> J(jacobians[0]);
      J.setZero();

      double Z_inv = 1.0 / Z;
      double Z2_inv = Z_inv * Z_inv;

      Eigen::Matrix<double, 2, 6> J_se3;

      J_se3 << fx * Z_inv, 0, -fx * X * Z2_inv,
          -fx * X * Y * Z2_inv, fx + fx * X * X * Z2_inv, -fx * Y * Z_inv,

          0, fy * Z_inv, -fy * Y * Z2_inv,
          -fy - fy * Y * Y * Z2_inv, fy * X * Y * Z2_inv, fy * X * Z_inv;

      idx = 0;
      for (int x = -1; x <= 1; x++)
      {
        for (int y = -1; y <= 1; y++)
        {
          double dx = 0.5 * (GetPixelValue(img2_, u + x + 1, v + y) -
                             GetPixelValue(img2_, u + x - 1, v + y));

          double dy = 0.5 * (GetPixelValue(img2_, u + x, v + y + 1) -
                             GetPixelValue(img2_, u + x, v + y - 1));

          Eigen::RowVector2d J_img(dx, dy);

          J.row(idx++) = -J_img * J_se3;
        }
      }
    }

    return true;
  }

private:
  const cv::Mat &img1_, &img2_;
  Eigen::Vector2d uv1_;
  double d1_;
  Eigen::Matrix3d K_, K_inv_;
};

int main(int argc, char **argv)
{

  cv::Mat left_img = cv::imread(left_file, 0);
  cv::Mat disparity_img = cv::imread(disparity_file, 0);

  // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
  cv::RNG rng;
  int nPoints = 2000;
  int boarder = 20;
  VecVector2d pixels_ref;
  vector<double> depth_ref;

  // generate pixels in ref and load depth data
  for (int i = 0; i < nPoints; i++)
  {
    int x = rng.uniform(boarder, left_img.cols - boarder); // don't pick pixels close to boarder
    int y = rng.uniform(boarder, left_img.rows - boarder); // don't pick pixels close to boarder
    int disparity = disparity_img.at<uchar>(y, x);
    if (disparity <= 0)
      continue;

    double depth = fx * baseline / disparity; // you know this is disparity to depth
    depth_ref.push_back(depth);
    pixels_ref.push_back(Eigen::Vector2d(x, y));
  }

  cv::Mat img1_show;
  cv::cvtColor(left_img, img1_show, cv::COLOR_GRAY2BGR);

  int valid_count = 0;
  for (size_t i = 0; i < pixels_ref.size(); ++i)
  {
    auto p_ref = pixels_ref[i];

    // 检查投影点是否在图像内
    if (p_ref[0] > 0 && p_ref[0] < left_img.cols &&
        p_ref[1] > 0 && p_ref[1] < left_img.rows)
    {
      // 绘制投影点（绿色圆点）
      cv::circle(img1_show, cv::Point2f(p_ref[0], p_ref[1]), 2, cv::Scalar(0, 250, 0), 2);
      valid_count++;
    }
  }
  cv::imshow("origin", img1_show);

  // estimates 01~05.png's pose using this information
  Sophus::SE3d T_cur_ref_original;
  Sophus::SE3d T_cur_ref_g2o;
  Sophus::SE3d T_cur_ref_ceres;
  for (int i = 1; i < 2; i++)
  { // 1~10

    char filename[100];
    sprintf(filename, "../%06d.png", i);
    // cv::Mat img = cv::imread((fmt_others % i).str(), 0);
    // cv::Mat img = cv::imread(std::format("../{0:06d}.png", i), 0);
    cv::Mat img = cv::imread(filename, 0);
    if (img.empty())
    {
      cerr << "Failed to load image: " << filename << endl;
      continue;
    }
    // try single layer by uncomment this line
    chrono::steady_clock::time_point t1, t2;
    chrono::duration<double> time_used;
    // try single layer by uncomment this line
    cout << "optimize by native g-n" << endl;
    t1 = chrono::steady_clock::now();
    DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref_original);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "native g-n optimization costs time: " << time_used.count() << " seconds." << endl;
    // DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref); // tj : 每次都从left_img开始估计，T_cur_ref会被更新为当前帧相对于left_img的变换
    cout << "optimize by g2o" << endl;
    t1 = chrono::steady_clock::now();
    DirectPoseEstimationSingleLayerG2O(left_img, img, pixels_ref, depth_ref, T_cur_ref_g2o);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "g2o optimization costs time: " << time_used.count() << " seconds." << endl;

    cout << "optimize by ceres" << endl;
    t1 = chrono::steady_clock::now();
    DirectPoseEstimationSingleLayerCeres(left_img, img, pixels_ref, depth_ref, T_cur_ref_ceres);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "ceres optimization costs time: " << time_used.count() << " seconds." << endl;

    while (cv::waitKey(0) != 'n')
      ;
  }
  return 0;
}

void DirectPoseEstimationSingleLayerCeres(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21)
{
  // ---------------------------------------------------
  // camera intrinsic
  // ---------------------------------------------------

  Eigen::Matrix3d K_eigen;
  K_eigen << fx, 0, cx,
      0, fy, cy,
      0, 0, 1;

  // ---------------------------------------------------
  // IMPORTANT:
  // use sophus se3 tangent directly
  //
  // camera = [tx ty tz rx ry rz]
  // where:
  //
  // T = exp(xi)
  // ---------------------------------------------------

  double camera[6];
  Eigen::Matrix<double, 6, 1> xi_init = T21.log();

  for (int i = 0; i < 6; i++)
    camera[i] = xi_init[i];

  // ---------------------------------------------------
  // build problem
  // ---------------------------------------------------

  ceres::Problem problem;
  // ⭐ 关键：将 camera 参数块绑定到 SE3Manifold
  problem.AddParameterBlock(camera, 6, new SE3Manifold());

  for (size_t i = 0; i < px_ref.size(); ++i)
  {
    if (depth_ref[i] <= 0)
      continue;

    ceres::CostFunction *cost_function =
        new PhotometricErrorAnalytic(
            px_ref[i],
            depth_ref[i],
            K_eigen,
            img1,
            img2);

    problem.AddResidualBlock(
        cost_function,
        new ceres::HuberLoss(5),
        camera);
  }

  // ---------------------------------------------------
  // solver options
  // ---------------------------------------------------

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 100;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.function_tolerance = 1e-6;
  options.gradient_tolerance = 1e-10;

  // ---------------------------------------------------
  // solve
  // ---------------------------------------------------

  ceres::Solver::Summary summary;

  auto t1 = chrono::steady_clock::now();

  ceres::Solve(options, &problem, &summary);

  auto t2 = chrono::steady_clock::now();

  chrono::duration<double> time_used =
      chrono::duration_cast<chrono::duration<double>>(t2 - t1);

  cout << "ceres optimization cost time: "
       << time_used.count()
       << " seconds."
       << endl;

  // ---------------------------------------------------
  // recover pose
  // ---------------------------------------------------

  Eigen::Matrix<double, 6, 1> xi_final;

  for (int i = 0; i < 6; i++)
    xi_final[i] = camera[i];

  T21 = Sophus::SE3d::exp(xi_final);

  cout << "pose from ceres:\n"
       << T21.matrix()
       << endl;

  // ---------------------------------------------------
  // visualization
  // ---------------------------------------------------

  VecVector2d projections;

  projections.reserve(px_ref.size());

  for (size_t i = 0; i < px_ref.size(); ++i)
  {
    auto uv1 = px_ref[i];
    auto d1 = depth_ref[i];

    if (d1 <= 0)
    {
      projections.push_back(
          Eigen::Vector2d(-1, -1));
      continue;
    }

    // back-project
    Eigen::Vector3d p1 =
        d1 *
        K_eigen.inverse() *
        Eigen::Vector3d(
            uv1[0],
            uv1[1],
            1);

    // transform
    Eigen::Vector3d p2 =
        T21 * p1;

    if (p2[2] <= 0)
    {
      projections.push_back(
          Eigen::Vector2d(-1, -1));
      continue;
    }

    // project
    Eigen::Vector3d uv2 =
        K_eigen * p2;

    uv2 /= uv2[2];

    projections.push_back(
        uv2.head<2>());
  }

  // ---------------------------------------------------
  // draw
  // ---------------------------------------------------

  cv::Mat img2_show;

  cv::cvtColor(
      img2,
      img2_show,
      cv::COLOR_GRAY2BGR);

  int valid_count = 0;

  for (size_t i = 0; i < px_ref.size(); ++i)
  {
    auto p_ref = px_ref[i];
    auto p_cur = projections[i];

    if (p_cur[0] > 0 &&
        p_cur[0] < img2.cols &&
        p_cur[1] > 0 &&
        p_cur[1] < img2.rows)
    {
      cv::circle(
          img2_show,
          cv::Point2f(p_cur[0], p_cur[1]),
          2,
          cv::Scalar(0, 250, 0),
          2);

      cv::line(
          img2_show,
          cv::Point2f(p_ref[0], p_ref[1]),
          cv::Point2f(p_cur[0], p_cur[1]),
          cv::Scalar(0, 250, 0),
          1);

      valid_count++;
    }
  }

  cout << "Visualized "
       << valid_count
       << " valid projections"
       << endl;

  cv::imshow("ceres projection", img2_show);
}

void DirectPoseEstimationSingleLayerG2O(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21)
{
  // =========================
  // g2o setup
  // =========================
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 9>>;
  // using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;
  using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
  // using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
  // auto solver = new g2o::OptimizationAlgorithmGaussNewton(
  //     std::make_unique<BlockSolverType>(
  //         std::make_unique<LinearSolverType>()));

  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(
          std::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // =========================
  // vertex
  // =========================
  VertexPose *vertex = new VertexPose();
  vertex->setId(0);
  vertex->setEstimate(T21);
  optimizer.addVertex(vertex);

  // =========================
  // camera intrinsics
  // =========================
  Eigen::Matrix3d K_eigen;
  K_eigen << fx, 0, cx,
      0, fy, cy,
      0, 0, 1;

  int edge_id = 0;
  int valid = 0;

  // =========================
  // edges
  // =========================
  for (size_t i = 0; i < px_ref.size(); i++)
  {
    if (depth_ref[i] <= 0)
      continue;

    auto uv = px_ref[i];
    double d = depth_ref[i];

    if (uv[0] < 1 || uv[0] >= img1.cols - 1 ||
        uv[1] < 1 || uv[1] >= img1.rows - 1)
      continue;

    EdgeDirectPoseOnly *edge =
        new EdgeDirectPoseOnly(uv, K_eigen, d, img1, img2);

    edge->setId(edge_id++);
    edge->setVertex(0, vertex);

    // ⭐ robust kernel
    auto rk = new g2o::RobustKernelHuber;
    rk->setDelta(5.0);
    edge->setRobustKernel(rk);

    // ⭐ 信息矩阵（标量）
    edge->setInformation(Eigen::Matrix<double, 9, 9>::Identity());

    optimizer.addEdge(edge);
    valid++;
  }

  std::cout << "valid edges: " << valid << std::endl;

  // =========================
  // optimize
  // =========================
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  T21 = vertex->estimate();

  std::cout << "g2o result:\n"
            << T21.matrix() << std::endl;

  VecVector2d projections;
  projections.reserve(px_ref.size());

  for (size_t i = 0; i < px_ref.size(); ++i)
  {
    auto uv1 = px_ref[i];
    auto d1 = depth_ref[i];

    // 检查有效性
    if (d1 <= 0)
    {
      projections.push_back(Eigen::Vector2d(-1, -1));
      continue;
    }

    // 计算3D点
    Eigen::Vector3d p1 = d1 * K_eigen.inverse() * Eigen::Vector3d(uv1[0], uv1[1], 1);
    // 变换到当前帧
    Eigen::Vector3d p2 = T21 * p1;
    // 投影到像素平面
    Eigen::Vector3d uv2 = K_eigen * p2;
    uv2 /= uv2[2];

    projections.push_back(uv2.head<2>());
  }

  // 可视化
  cv::Mat img2_show;
  cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);

  int valid_count = 0;
  for (size_t i = 0; i < px_ref.size(); ++i)
  {
    auto p_ref = px_ref[i];
    auto p_cur = projections[i];

    // 检查投影点是否在图像内
    if (p_cur[0] > 0 && p_cur[0] < img2.cols &&
        p_cur[1] > 0 && p_cur[1] < img2.rows)
    {
      // 绘制投影点（绿色圆点）
      cv::circle(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 2, cv::Scalar(0, 250, 0), 2);
      // 绘制连线（从参考点到投影点）
      cv::line(img2_show,
               cv::Point2f(p_ref[0], p_ref[1]),
               cv::Point2f(p_cur[0], p_cur[1]),
               cv::Scalar(0, 250, 0), 1);
      valid_count++;
    }
  }

  cout << "Visualized " << valid_count << " valid projections" << endl;

  cv::imshow("g2o projection", img2_show);
}

void DirectPoseEstimationSingleLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21)
{

  const int iterations = 10;
  double cost = 0, lastCost = 0;
  auto t1 = chrono::steady_clock::now();
  JacobianAccumulator jaco_accu(img1, img2, px_ref, depth_ref, T21);

  for (int iter = 0; iter < iterations; iter++)
  {
    jaco_accu.reset();
    cv::parallel_for_(cv::Range(0, px_ref.size()),
                      std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1)); // tj : 第一个参数是this, 第二个参数是range, 相当于调用accumulate_jacobian(jaco_accu, std::placeholders::_1)
    Matrix6d H = jaco_accu.hessian();
    Vector6d b = jaco_accu.bias();

    // solve update and put it into estimation
    Vector6d update = H.ldlt().solve(b);
    ;
    T21 = Sophus::SE3d::exp(update) * T21;
    cost = jaco_accu.cost_func();

    if (std::isnan(update[0]))
    {
      // sometimes occurred when we have a black or white patch and H is irreversible
      cout << "update is nan" << endl;
      break;
    }
    if (iter > 0 && cost > lastCost)
    {
      cout << "cost increased: " << cost << ", " << lastCost << endl;
      break;
    }
    if (update.norm() < 1e-3)
    {
      // converge
      break;
    }

    lastCost = cost;
    cout << "iteration: " << iter << ", cost: " << cost << endl;
  }

  cout << "T21 = \n"
       << T21.matrix() << endl;
  auto t2 = chrono::steady_clock::now();
  auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "direct method for single layer: " << time_used.count() << endl;

  // plot the projected pixels here
  cv::Mat img2_show;
  cv::cvtColor(img2, img2_show, cv::COLOR_GRAY2BGR);
  VecVector2d projection = jaco_accu.projected_points();
  for (size_t i = 0; i < px_ref.size(); ++i)
  {
    auto p_ref = px_ref[i];
    auto p_cur = projection[i];
    if (p_cur[0] > 0 && p_cur[1] > 0 && p_cur[0] < img2.cols && p_cur[1] < img2.rows)
    {
      cv::circle(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img2_show, cv::Point2f(p_ref[0], p_ref[1]), cv::Point2f(p_cur[0], p_cur[1]),
               cv::Scalar(0, 250, 0));
    }
  }
  cv::imshow("g-n", img2_show);
}

void JacobianAccumulator::accumulate_jacobian(const cv::Range &range)
{

  // parameters
  const int half_patch_size = 1;
  int cnt_good = 0;
  Matrix6d hessian = Matrix6d::Zero();
  Vector6d bias = Vector6d::Zero();
  double cost_tmp = 0;

  for (size_t i = range.start; i < range.end; i++)
  {

    // compute the projection in the second image
    Eigen::Vector3d point_ref =
        depth_ref[i] * Eigen::Vector3d((px_ref[i][0] - cx) / fx, (px_ref[i][1] - cy) / fy, 1);
    Eigen::Vector3d point_cur = T21 * point_ref;
    if (point_cur[2] < 0) // depth invalid
      continue;

    float u = fx * point_cur[0] / point_cur[2] + cx, v = fy * point_cur[1] / point_cur[2] + cy;
    if (u < half_patch_size || u > img2.cols - half_patch_size || v < half_patch_size ||
        v > img2.rows - half_patch_size)
      continue;

    projection[i] = Eigen::Vector2d(u, v);
    double X = point_cur[0], Y = point_cur[1], Z = point_cur[2],
           Z2 = Z * Z, Z_inv = 1.0 / Z, Z2_inv = Z_inv * Z_inv;
    cnt_good++;

    // and compute error and jacobian
    for (int x = -half_patch_size; x <= half_patch_size; x++)
      for (int y = -half_patch_size; y <= half_patch_size; y++)
      {

        double error = GetPixelValue(img1, px_ref[i][0] + x, px_ref[i][1] + y) -
                       GetPixelValue(img2, u + x, v + y);
        Matrix26d J_pixel_xi;
        Eigen::Vector2d J_img_pixel;

        J_pixel_xi(0, 0) = fx * Z_inv;
        J_pixel_xi(0, 1) = 0;
        J_pixel_xi(0, 2) = -fx * X * Z2_inv;
        J_pixel_xi(0, 3) = -fx * X * Y * Z2_inv;
        J_pixel_xi(0, 4) = fx + fx * X * X * Z2_inv;
        J_pixel_xi(0, 5) = -fx * Y * Z_inv;

        J_pixel_xi(1, 0) = 0;
        J_pixel_xi(1, 1) = fy * Z_inv;
        J_pixel_xi(1, 2) = -fy * Y * Z2_inv;
        J_pixel_xi(1, 3) = -fy - fy * Y * Y * Z2_inv;
        J_pixel_xi(1, 4) = fy * X * Y * Z2_inv;
        J_pixel_xi(1, 5) = fy * X * Z_inv;

        J_img_pixel = Eigen::Vector2d(
            0.5 * (GetPixelValue(img2, u + 1 + x, v + y) - GetPixelValue(img2, u - 1 + x, v + y)),
            0.5 * (GetPixelValue(img2, u + x, v + 1 + y) - GetPixelValue(img2, u + x, v - 1 + y)));

        // total jacobian
        Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();

        hessian += J * J.transpose();
        bias += -error * J;
        cost_tmp += error * error;
      }
  }

  if (cnt_good)
  {
    // set hessian, bias and cost
    unique_lock<mutex> lck(hessian_mutex); // tj : lock the mutex to protect the shared variables H, b, cost
    H += hessian;
    b += bias;
    cost += cost_tmp / cnt_good;
  }
}

void DirectPoseEstimationMultiLayer(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21)
{

  // parameters
  int pyramids = 4;
  double pyramid_scale = 0.5;
  double scales[] = {1.0, 0.5, 0.25, 0.125};

  // create pyramids
  vector<cv::Mat> pyr1, pyr2; // image pyramids
  for (int i = 0; i < pyramids; i++)
  {
    if (i == 0)
    {
      pyr1.push_back(img1);
      pyr2.push_back(img2);
    }
    else
    {
      cv::Mat img1_pyr, img2_pyr;
      cv::resize(pyr1[i - 1], img1_pyr,
                 cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
      cv::resize(pyr2[i - 1], img2_pyr,
                 cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
      pyr1.push_back(img1_pyr);
      pyr2.push_back(img2_pyr);
    }
  }

  double fxG = fx, fyG = fy, cxG = cx, cyG = cy; // backup the old values
  for (int level = pyramids - 1; level >= 0; level--)
  {
    VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
    for (auto &px : px_ref)
    {
      px_ref_pyr.push_back(scales[level] * px);
    }

    // scale fx, fy, cx, cy in different pyramid levels
    fx = fxG * scales[level];
    fy = fyG * scales[level];
    cx = cxG * scales[level];
    cy = cyG * scales[level];
    DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
  }
}
