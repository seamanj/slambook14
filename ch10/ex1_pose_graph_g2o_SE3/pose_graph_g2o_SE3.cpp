#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

/************************************************
 * 本程序演示如何用g2o solver进行位姿图优化
 * sphere.g2o是人工生成的一个Pose graph，我们来优化它。
 * 尽管可以直接通过load函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解
 * 这里使用g2o/types/slam3d/中的SE3表示位姿，它实质上是四元数而非李代数.
 * **********************************************/


/*
G2O文件格式：
*****************************************************
VERTEX_SE3:QUAT id x y z qx qy qz qw
VERTEX_SE3:QUAT 16 0.883238 -1.95252 99.744 -0.372324 -0.564041 -0.380507 0.631226
顶点 ID = 16
位置：(0.883, -1.953, 99.744)
四元数：(-0.372, -0.564, -0.381, 0.631)
*****************************************************
EDGE_SE3:QUAT id1 id2 dx dy dz dqx dqy dqz dqw inf11 inf12 ... inf66
EDGE_SE3:QUAT 769 819 -0.217415 -0.0102919 -6.28424 0.00345993 0.0183954 -0.000466133 0.999825 10000 0 0 0 0 0 10000 0 0 0 0 10000 0 0 0 40000 0 0 40000 0 40000
连接顶点 769 和 819
相对平移：(-0.217, -0.010, -6.284)
相对旋转四元数：(0.00346, 0.01840, -0.000466, 0.999825)
信息矩阵（6x6，只给出上三角）：
text
[10000, 0,    0,    0,    0,    0;
 0,    10000, 0,    0,    0,    0;
 0,    0,    10000, 0,    0,    0;
 0,    0,    0,    40000, 0,    0;
 0,    0,    0,    0,    40000, 0;
 0,    0,    0,    0,    0,    40000]
其中前3个对角元对应平移（10000），后3个对应旋转（40000）
*****************************************************
*/

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

    // 设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            // SE3 顶点
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            // SE3-SE3 边
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            int idx1, idx2;     // 关联的两个顶点
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;
    optimizer.save("result.g2o");

    return 0;
}