#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <fstream>
#include <vector>
#include <iostream>
#include <map>
#include <Eigen/Core>
#include <Eigen/Cholesky>

using Sophus::SE3d;
using Sophus::SO3d;
using Sophus::Vector6d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Matrix3d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// ==================== JRInv 函数 ====================
Matrix6d JRInv(const SE3d &e) {
    Matrix6d J;
    J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Matrix3d::Zero();
    J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());
    J = J * 0.5 + Matrix6d::Identity();
    return J; 
}

// ==================== SE3 流形（李代数加法） ====================
class SE3Manifold : public ceres::Manifold {
public:
    // 李代数是 6 维
    int AmbientSize() const override { return 6; }
    int TangentSize() const override { return 6; }

    // x_plus_delta = exp(delta) * x
    bool Plus(const double *x,
              const double *delta,
              double *x_plus_delta) const override {
        Eigen::Map<const Vector6d> xi(x);
        Eigen::Map<const Vector6d> dx(delta);

        SE3d T = SE3d::exp(xi);
        SE3d dT = SE3d::exp(dx);
        SE3d T_new = dT * T;

        Eigen::Map<Vector6d> xi_new(x_plus_delta);
        xi_new = T_new.log();
        return true;
    }

    // y - x（切空间中的差）
    bool Minus(const double *y,
               const double *x,
               double *y_minus_x) const override {
        Eigen::Map<const Vector6d> xi1(y);
        Eigen::Map<const Vector6d> xi2(x);

        SE3d T1 = SE3d::exp(xi1);
        SE3d T2 = SE3d::exp(xi2);
        SE3d dT = T2.inverse() * T1;

        Eigen::Map<Vector6d> dx(y_minus_x);
        dx = dT.log();
        return true;
    }

    // Plus 的雅可比（默认单位矩阵即可）
    bool PlusJacobian(const double *x,
                      double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }

    // Minus 的雅可比（默认单位矩阵即可）
    bool MinusJacobian(const double *x,
                       double *jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobian);
        J.setIdentity();
        return true;
    }
};

// ==================== 相对位姿误差边 ====================
class EdgeSE3LieAlgebra : public ceres::SizedCostFunction<6, 6, 6> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeSE3LieAlgebra(const SE3d& measurement, const Matrix6d& information)
        : measurement_(measurement), information_(information) {
        
        Eigen::LLT<Matrix6d> llt(information_);
        sqrt_info_ = llt.matrixL();
    }

    virtual bool Evaluate(double const* const* parameters,
                         double* residuals,
                         double** jacobians) const override {
        
        Eigen::Map<const Vector6d> xi_i(parameters[0]);
        Eigen::Map<const Vector6d> xi_j(parameters[1]);
        
        SE3d Ti = SE3d::exp(xi_i);
        SE3d Tj = SE3d::exp(xi_j);
        
        // 误差计算（与 g2o 一致）
        SE3d error_se3 = measurement_.inverse() * Ti.inverse() * Tj;
        Vector6d error_vec = error_se3.log();
        
        // 加权残差
        Vector6d weighted_error = sqrt_info_ * error_vec;
        
        for (int i = 0; i < 6; ++i) {
            residuals[i] = weighted_error[i];
        }
        
        // 雅可比计算
        if (jacobians != nullptr) {
            Matrix6d J = JRInv(error_se3);
            
            if (jacobians[0] != nullptr) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J_i(jacobians[0]);
                // 注意：由于使用了 Manifold，Ceres 会自动处理，这里不需要额外调整
                J_i = sqrt_info_ * (-J * Tj.inverse().Adj());
            }
            
            if (jacobians[1] != nullptr) {
                Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J_j(jacobians[1]);
                J_j = sqrt_info_ * (J * Tj.inverse().Adj());
            }
        }
        
        return true;
    }

private:
    const SE3d measurement_;
    const Matrix6d information_;
    Matrix6d sqrt_info_;
};

// ==================== 读取顶点 ====================
SE3d readVertex(std::istream& is) {
    double data[7];
    for (int i = 0; i < 7; ++i) {
        is >> data[i];
    }
    Quaterniond q(data[6], data[3], data[4], data[5]);
    q.normalize();
    return SE3d(q, Vector3d(data[0], data[1], data[2]));
}

// ==================== 读取信息矩阵 ====================
Matrix6d readInformation(std::istream& is) {
    Matrix6d info = Matrix6d::Zero();
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            double mij;
            is >> mij;
            info(i, j) = mij;
            info(j, i) = mij;
        }
    }
    return info;
}

// ==================== 保存优化结果 ====================
void saveToG2O(const std::string& filename,
               const std::map<int, SE3d>& optimized_poses,
               const std::vector<std::tuple<int, int, SE3d, Matrix6d>>& edges) {
    
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    std::cout << "Saving optimization results to " << filename << " ..." << std::endl;
    
    for (const auto& pose : optimized_poses) {
        int id = pose.first;
        SE3d T = pose.second;
        
        Vector3d t = T.translation();
        Quaterniond q = T.unit_quaternion();
        
        fout << "VERTEX_SE3:QUAT " << id << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << std::endl;
    }
    
    for (const auto& edge : edges) {
        int id1 = std::get<0>(edge);
        int id2 = std::get<1>(edge);
        SE3d Tij = std::get<2>(edge);
        Matrix6d info = std::get<3>(edge);
        
        Vector3d t = Tij.translation();
        Quaterniond q = Tij.unit_quaternion();
        
        fout << "EDGE_SE3:QUAT " << id1 << " " << id2 << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        
        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                fout << info(i, j) << " ";
            }
        }
        fout << std::endl;
    }
    
    fout.close();
    std::cout << "Saved to " << filename << std::endl;
}
// ==================== 保存优化结果（包含更新后的 edges） ====================
void saveToG2OWithUpdatedEdges(const std::string& filename,
                               const std::map<int, SE3d>& optimized_poses,
                               const std::vector<std::tuple<int, int, SE3d, Matrix6d>>& original_edges) {
    
    std::ofstream fout(filename);
    if (!fout.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    std::cout << "Saving optimization results with UPDATED edges to " << filename << " ..." << std::endl;
    
    // 保存优化后的顶点
    for (const auto& pose : optimized_poses) {
        int id = pose.first;
        SE3d T = pose.second;
        
        Vector3d t = T.translation();
        Quaterniond q = T.unit_quaternion();
        
        fout << "VERTEX_SE3:QUAT " << id << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << std::endl;
    }
    
    // 保存更新后的边（基于优化后的位姿重新计算相对位姿）
    int edge_count = 0;
    for (const auto& edge : original_edges) {
        int id1 = std::get<0>(edge);
        int id2 = std::get<1>(edge);
        Matrix6d info = std::get<3>(edge);  // 信息矩阵保持不变
        
        // 检查两个顶点是否都存在
        if (optimized_poses.find(id1) == optimized_poses.end() ||
            optimized_poses.find(id2) == optimized_poses.end()) {
            std::cerr << "Warning: vertex " << id1 << " or " << id2 << " not found, skipping edge" << std::endl;
            continue;
        }
        
        // 【关键】基于优化后的位姿重新计算相对位姿
        // T_ij_optimized = T_i^{-1} * T_j
        SE3d Ti = optimized_poses.at(id1);
        SE3d Tj = optimized_poses.at(id2);
        SE3d updated_measurement = Ti.inverse() * Tj;
        
        // 也可以计算误差（与原始测量的差异）
        SE3d original_measurement = std::get<2>(edge);
        SE3d error = original_measurement.inverse() * updated_measurement;
        Vector6d error_vec = error.log();
        
        // 输出更新后的相对位姿
        Vector3d t = updated_measurement.translation();
        Quaterniond q = updated_measurement.unit_quaternion();
        
        fout << "EDGE_SE3:QUAT " << id1 << " " << id2 << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        
        // 保存信息矩阵（与原信息矩阵相同）
        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                fout << info(i, j) << " ";
            }
        }
        fout << std::endl;
        
        edge_count++;
        
        // 可选：打印误差统计
        if (edge_count <= 5) {
            std::cout << "Edge " << id1 << "->" << id2 
                      << ": error norm = " << error_vec.norm() << std::endl;
        }
    }
    
    fout.close();
    std::cout << "Saved " << optimized_poses.size() << " vertices and " 
              << edge_count << " updated edges to " << filename << std::endl;
}
// ==================== 主函数 ====================
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: pose_graph_ceres sphere.g2o" << std::endl;
        return 1;
    }
    
    std::ifstream fin(argv[1]);
    if (!fin) {
        std::cout << "File " << argv[1] << " does not exist." << std::endl;
        return 1;
    }
    
    std::map<int, SE3d> initial_poses;
    std::map<int, Vector6d> param_blocks;
    std::vector<std::tuple<int, int, SE3d, Matrix6d>> edges;
    
    std::string tag;
    int vertex_count = 0;
    int edge_count = 0;
    
    std::cout << "Reading from g2o file..." << std::endl;
    
    while (fin >> tag) {
        if (tag == "VERTEX_SE3:QUAT") {
            int id;
            fin >> id;
            SE3d pose = readVertex(fin);
            initial_poses[id] = pose;
            
            Vector6d tangent = pose.log();
            param_blocks[id] = tangent;
            vertex_count++;
        }
        else if (tag == "EDGE_SE3:QUAT") {
            int id1, id2;
            fin >> id1 >> id2;
            SE3d measurement = readVertex(fin);
            Matrix6d information = readInformation(fin);
            edges.emplace_back(id1, id2, measurement, information);
            edge_count++;
        }
        
        if (!fin.good()) break;
    }
    
    fin.close();
    
    std::cout << "Loaded " << vertex_count << " vertices and " 
              << edge_count << " edges" << std::endl;
    
    std::cout << "\nInitial poses (first 3):" << std::endl;
    int count = 0;
    for (const auto& pose : initial_poses) {
        if (count++ >= 3) break;
        std::cout << "ID " << pose.first << ": t = [" 
                  << pose.second.translation().transpose() << "]" << std::endl;
    }
    
    // ========== 创建 Ceres 问题 ==========
    ceres::Problem problem;
    
    // 创建流形对象
    SE3Manifold* se3_manifold = new SE3Manifold();
    
    // 分配连续内存用于参数
    std::vector<double> param_memory;
    param_memory.resize(vertex_count * 6);
    std::map<int, double*> param_ptrs;
    
    int idx = 0;
    for (auto& param : param_blocks) {
        double* ptr = param_memory.data() + idx * 6;
        Eigen::Map<Vector6d> map(ptr);
        map = param.second;
        param_ptrs[param.first] = ptr;
        
        // 【关键】添加参数块并使用 SE3 流形
        problem.AddParameterBlock(ptr, 6, se3_manifold);
        idx++;
    }
    
    // 添加边（残差块）
    int skipped = 0;
    for (const auto& edge : edges) {
        int id1 = std::get<0>(edge);
        int id2 = std::get<1>(edge);
        
        if (param_ptrs.find(id1) == param_ptrs.end() ||
            param_ptrs.find(id2) == param_ptrs.end()) {
            skipped++;
            continue;
        }
        
        SE3d measurement = std::get<2>(edge);
        Matrix6d information = std::get<3>(edge);
        
        ceres::CostFunction* cost_function = new EdgeSE3LieAlgebra(measurement, information);
        problem.AddResidualBlock(cost_function, nullptr,
                                 param_ptrs[id1], param_ptrs[id2]);
    }
    
    if (skipped > 0) {
        std::cout << "Skipped " << skipped << " edges due to missing vertices" << std::endl;
    }
    
    // ========== 固定第一个顶点 ==========
    if (!param_ptrs.empty()) {
        auto first_vertex = param_ptrs.begin();
        problem.SetParameterBlockConstant(first_vertex->second);
        std::cout << "Fixed vertex ID: " << first_vertex->first << std::endl;
    }
    
    // ========== 设置求解器选项 ==========
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    options.num_threads = 4;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;
    
    // ========== 求解 ==========
    ceres::Solver::Summary summary;
    std::cout << "\nStarting optimization..." << std::endl;
    ceres::Solve(options, &problem, &summary);
    
    std::cout << summary.BriefReport() << std::endl;
    
    if (summary.termination_type == ceres::FAILURE) {
        std::cerr << "Optimization failed!" << std::endl;
        std::cerr << summary.FullReport() << std::endl;
        return 1;
    }
    
    // ========== 收集优化后的位姿 ==========
    std::map<int, SE3d> optimized_poses;
    for (const auto& param : param_ptrs) {
        int id = param.first;
        Vector6d xi = Eigen::Map<Vector6d>(param.second);
        optimized_poses[id] = SE3d::exp(xi);
    }
    

/*
如果你想输出一个可用于可视化的文件，当前方式完全正确。

如果你想输出一个包含优化后相对约束的图（比如为了某种闭环检测），那才需要重新计算 edges：
*/

    // 1. 标准保存：只保存优化后的顶点，edges 保持原样
    saveToG2O("result_ceres_standard.g2o", optimized_poses, edges);
    
    // 2. 保存更新后的 edges（基于优化后的位姿重新计算）
    saveToG2OWithUpdatedEdges("result_ceres_with_updated_edges.g2o", optimized_poses, edges);
    
    std::cout << "\nDone!" << std::endl;
    
    return 0;
}