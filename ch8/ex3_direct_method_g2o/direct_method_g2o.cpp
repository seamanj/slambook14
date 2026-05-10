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
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <g2o/core/robust_kernel_impl.h>
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

    /// left multiplication on SE3
    virtual void oplusImpl(const double *update) override
    {
        Eigen::Matrix<double, 6, 1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(istream &in) override { return true; }

    virtual bool write(ostream &out) const override { return true; }
};
/// g2o edge with 3x3 patch
class EdgeDirectPoseOnly : public g2o::BaseUnaryEdge<9, Eigen::Matrix<double, 9, 1>, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 使用 const cv::Mat& 引用传递
    EdgeDirectPoseOnly(const Eigen::Vector2d &uv1, const Eigen::Matrix3d &K, double d1,
                       const cv::Mat &img1, const cv::Mat &img2)
        : uv1_(uv1), K_(K), d1_(d1), img1_(img1), img2_(img2)
    {
        // 计算3D点
        p1_ = d1_ * K_.inverse() * Eigen::Vector3d(uv1_[0], uv1_[1], 1);

        // 提取3x3 patch的测量值（参考帧的9个像素）
        // measurement_.resize(9);

         measurement_.setZero();   // ⭐ FIX 1 (NO resize)

        int idx = 0;
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                float px = uv1_[0] + x;
                float py = uv1_[1] + y;
                // 边界检查
                if (px >= 1 && px < img1_.cols - 1 && py >= 1 && py < img1_.rows - 1)
                {
                    measurement_[idx] = GetPixelValue(img1_, px, py);
                }
                else
                {
                    measurement_[idx] = 0;
                }
                idx++;
            }
        }
    }

    virtual void computeError() override
    {
        setLevel(0);
        const VertexPose *T21 = static_cast<const VertexPose *>(_vertices[0]);

        Eigen::Vector3d p2 = T21->estimate() * p1_;
        Eigen::Vector3d uv2 = K_ * p2;
        uv2 /= uv2[2];

        // 检查投影点是否在图像内（考虑patch大小）
        if (uv2[0] < 1 || uv2[0] >= img2_.cols - 1 ||
            uv2[1] < 1 || uv2[1] >= img2_.rows - 1)
        {
            _error.setZero();
            setLevel(1);
            return;
        }

        // 计算3x3 patch的误差
        int idx = 0;
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                float u = uv2[0] + x;
                float v = uv2[1] + y;

                // 边界检查
                if (u >= 1 && u < img2_.cols - 1 && v >= 1 && v < img2_.rows - 1)
                {
                    double pixel_value = GetPixelValue(img2_, u, v);
                    _error[idx] = measurement_[idx] - pixel_value;
                }
                else
                {
                    _error[idx] = 0;
                }
                idx++;
            }
        }
    }

    virtual void linearizeOplus() override
    {
        VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T21 = pose->estimate();

        Eigen::Vector3d p2 = T21 * p1_;
        Eigen::Vector3d uv2 = K_ * p2;
        uv2 /= uv2[2];

        if (uv2[0] < 1 || uv2[0] >= img2_.cols - 1 ||
            uv2[1] < 1 || uv2[1] >= img2_.rows - 1)
        {
            _jacobianOplusXi = Eigen::Matrix<double, 9, 6>::Zero();
            setLevel(1);
            return;
        }

        double fx = K_(0, 0);
        double fy = K_(1, 1);
        double X = p2[0], Y = p2[1], Z = p2[2];
        double Z_inv = 1.0 / Z;
        double Z2_inv = Z_inv * Z_inv;

        // 像素对位姿的雅可比 (2x6)
        Eigen::Matrix<double, 2, 6> J_pixel_xi;
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

        // 对patch内每个像素计算雅可比
        _jacobianOplusXi = Eigen::Matrix<double, 9, 6>::Zero();

        int idx = 0;
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                float u = uv2[0] + x;
                float v = uv2[1] + y;

                if (u >= 1 && u < img2_.cols - 1 && v >= 1 && v < img2_.rows - 1)
                {
                    // 图像梯度
                    Eigen::Vector2d J_img_pixel;
                    J_img_pixel(0) = 0.5 * (GetPixelValue(img2_, u + 1, v) - GetPixelValue(img2_, u - 1, v));
                    J_img_pixel(1) = 0.5 * (GetPixelValue(img2_, u, v + 1) - GetPixelValue(img2_, u, v - 1));

                    // 雅可比：1x6 = (1x2) * (2x6)
                    _jacobianOplusXi.row(idx) = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();
                }
                else
                {
                    _jacobianOplusXi.row(idx).setZero();
                }
                idx++;
            }
        }
    }

    bool read(istream &in) override { return true; }
    bool write(ostream &out) const override { return true; }

protected:
    Eigen::Matrix3d K_;
    const cv::Mat &img1_; // 引用传递，不拷贝
    const cv::Mat &img2_; // 引用传递，不拷贝
    Eigen::Vector2d uv1_;
    double d1_;
    Eigen::Vector3d p1_;
    Eigen::Matrix<double, 9, 1> measurement_; // 9维测量向量
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
        double depth = fx * baseline / disparity; // you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // estimates 01~05.png's pose using this information
    Sophus::SE3d T_cur_ref_original;
    Sophus::SE3d T_cur_ref_g2o;

    for (int i = 1; i < 6; i++)
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
        DirectPoseEstimationSingleLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref_original);
        // DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref); // tj : 每次都从left_img开始估计，T_cur_ref会被更新为当前帧相对于left_img的变换
        DirectPoseEstimationSingleLayerG2O(left_img, img, pixels_ref, depth_ref, T_cur_ref_g2o);
    }
    return 0;
}
void DirectPoseEstimationSingleLayerG2O(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const VecVector2d &px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21)
{
    // 构建图优化
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);

    // vertex
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(T21);
    optimizer.addVertex(vertex_pose);

    // K
    Eigen::Matrix3d K_eigen;
    K_eigen << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // edges - 使用patch
    int index = 1;
    int valid_edges = 0;
    for (size_t i = 0; i < px_ref.size(); ++i)
    {
        auto uv1 = px_ref[i];
        auto d1 = depth_ref[i];

        // 检查有效性
        if (d1 <= 0)
            continue;
        if (uv1[0] < 1 || uv1[0] >= img1.cols - 1 || uv1[1] < 1 || uv1[1] >= img1.rows - 1)
            continue;

        // 创建边，传入img1和img2
        EdgeDirectPoseOnly *edge = new EdgeDirectPoseOnly(uv1, K_eigen, d1, img1, img2);
        edge->setId(index++);
        edge->setVertex(0, vertex_pose);
        // 不需要单独setMeasurement，因为在构造函数中已经提取了patch
        edge->setInformation(Eigen::Matrix<double, 9, 9>::Identity()); // 9x9信息矩阵
        auto rk = new g2o::RobustKernelHuber;
        rk->setDelta(5.0);
        edge->setRobustKernel(rk);
        optimizer.addEdge(edge);
        valid_edges++;
    }

    cout << "Added " << valid_edges << " edges with 3x3 patch" << endl;

    if (valid_edges == 0)
    {
        cout << "No valid edges, skipping optimization" << endl;
        return;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
optimizer.initializeOptimization();

optimizer.optimize(10);


    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "g2o optimization costs time: " << time_used.count() << " seconds." << endl;
    cout << "pose estimated by g2o =\n"
         << vertex_pose->estimate().matrix() << endl;

    T21 = vertex_pose->estimate();

    VecVector2d projections;
    projections.reserve(px_ref.size());
    
    for (size_t i = 0; i < px_ref.size(); ++i)
    {
        auto uv1 = px_ref[i];
        auto d1 = depth_ref[i];
        
        // 检查有效性
        if (d1 <= 0) {
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
    cv::waitKey(0); 
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
