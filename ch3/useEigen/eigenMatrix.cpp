#include <iostream>
using namespace std;
#include <chrono>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 100

/****************************
* 本程序演示了 Eigen 基本类型的使用
****************************/

int main( int argc, char** argv )
{
    // Eigen 中所有向量和矩阵都是Eigen::Matrix，它是一个模板类。它的前三个参数为：数据类型，行，列
    // 声明一个2*3的float矩阵
    Eigen::Matrix<float, 2, 3> matrix_23;

    // 同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是Eigen::Matrix
    // 例如 Vector3d 实质上是 Eigen::Matrix<double, 3, 1>，即三维向量
    Eigen::Vector3d v_3d;
	// 这是一样的
    Eigen::Matrix<float,3,1> vd_3d;

    // Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); //初始化为零
    // 如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_dynamic;
    // 更简单的
    Eigen::MatrixXd matrix_x;
    // 这种类型还有很多，我们不一一列举

    // 下面是对Eigen阵的操作
    // 输入数据（初始化）
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // 输出
    cout << matrix_23 << endl;

    // 用()访问矩阵中的元素
    for (int i=0; i<2; i++) {
        for (int j=0; j<3; j++)
            cout<<matrix_23(i,j)<<"\t";
        cout<<endl;
    }

    // 矩阵和向量相乘（实际上仍是矩阵和矩阵）
    v_3d << 3, 2, 1;
    vd_3d << 4,5,6;
    // 但是在Eigen里你不能混合两种不同类型的矩阵，像这样是错的
    // Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    // 应该显式转换
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;

    Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << result2 << endl;

    // 同样你不能搞错矩阵的维度
    // 试着取消下面的注释，看看Eigen会报什么错
    // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

    // 一些矩阵运算
    // 四则运算就不演示了，直接用+-*/即可。
    matrix_33 = Eigen::Matrix3d::Random();      // 随机数矩阵
    cout << matrix_33 << endl << endl;

    cout << matrix_33.transpose() << endl;      // 转置
    cout << matrix_33.sum() << endl;            // 各元素和
    cout << matrix_33.trace() << endl;          // 迹
    cout << 10*matrix_33 << endl;               // 数乘
    cout << matrix_33.inverse() << endl;        // 逆
    cout << matrix_33.determinant() << endl;    // 行列式

    // 特征值
    // 实对称矩阵可以保证对角化成功
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver ( matrix_33.transpose()*matrix_33 );
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

    // 解方程
    // 我们求解 matrix_NN * x = v_Nd 这个方程
    // N的大小在前边的宏里定义，它由随机数生成
    // 直接求逆自然是最直接的，但是求逆运算量大

    Eigen::Matrix< double, MATRIX_SIZE, MATRIX_SIZE > matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    Eigen::Matrix< double, MATRIX_SIZE,  1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random( MATRIX_SIZE,1 );

    auto start = chrono::high_resolution_clock::now();
    // 直接求逆
    Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse()*v_Nd;
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    cout <<"time use in normal inverse is " << duration.count() << "μs"<< endl;
    cout << "x = \n" << x.transpose() << endl;
	// 通常用矩阵分解来求，例如QR分解，速度会快很多
    auto qr_start = chrono::high_resolution_clock::now();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    auto qr_end = chrono::high_resolution_clock::now();
    auto qr_duration = chrono::duration_cast<chrono::microseconds>(qr_end - qr_start);
    cout <<"time use in Qr decomposition is " << qr_duration.count() << "μs" << endl;
    cout << "x = \n" << x.transpose() << endl;

    return 0;
}


/*

方法	                时间复杂度	            数值稳定性	        适用矩阵
inverse()	            O(n³)	                差	            小规模良态方阵
colPivHouseholderQr()	O(n³)	                好	            方阵、最小二乘
fullPivHouseholderQr()	O(n³)	                最好	        严重病态矩阵
llt() (Cholesky)	    O(n³/3)	                好	            对称正定
bdcSvd()	            O(n³)	                最              任意矩阵（最慢
*/

/*
QR	A = QR	O(n³)	高	最小二乘、病态矩阵
LU	A = LU	O(n³/3)	中	一般方阵
Cholesky	A = LLᵀ	O(n³/6)	高	对称正定矩阵
SVD	A = UΣVᵀ	O(n³)	最高	奇异、最小二乘
*/



/*
分解方法	        分解形式	时间复杂度	相对速度	    数值稳定性	内存占用
LU分解	            A = LU	    O(n³/3)	    最快 (1x)	    低	    低
Cholesky (LLT)	    A = LLᵀ	    O(n³/6)	    最快 (0.5x) 	高	    最低
LDLT分解	        A = LDLᵀ	O(n³/6) 	很快 (0.6x)	    高	    低
QR (householder)	A = QR	    O(2n³/3)	慢 (2-3x)	    高	    中
QR (列主元)	        AP = QR	    O(2n³/3)	较慢 (3-4x)	    较高	中
QR (全主元)	        QAP = R	    O(2n³/3)	慢 (5-8x)	    最高	高
SVD (稠密)	        A = UΣVᵀ	O(4n³/3)	最慢 (10-20x)	最高	最高



分解方法	        方阵求解	最小二乘	病态矩阵	奇异矩阵	秩亏矩阵	对称正定
LU分解	            ✅	        ❌	    ❌	        ❌	        ❌	    ❌
Cholesky (LLT)	    ✅	        ❌	    ✅	        ❌	        ❌	    ✅
LDLT分解	        ✅	        ❌	    ✅	        ❌	        ❌	    ✅
QR (householder)	✅	        ✅	    ✅	        ❌	        ❌	    ❌
QR (列主元)	        ✅	        ✅	    ✅	         ✅	        ✅	    ❌
QR (全主元)	        ✅	        ✅	    ✅✅	       ✅✅	     ✅✅	    ❌
SVD (稠密)	        ✅	        ✅✅	  ✅✅✅	    ✅✅✅	    ✅✅✅	  ❌


*/