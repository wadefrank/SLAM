#include <iostream>

#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main(int argc, char **argv)
{
    // Eigen::Matrix：Eigen中所有向量和矩阵都是Eigen::Matrix，它是一个模板类，前三个参数分别为： 数据类型，行，列
    Eigen::Matrix<float, 2, 3> matrix_23f;
    Eigen::Matrix<float, 3, 1> vector_3f;

    // 内置类型：同时Eigen通过typedef提供了许多内置类型，不过底层仍然是Eigen::Matrix
    Eigen::Vector3d vector_3d;  // Eigen::Matrix<double, 3, 1> vector_3d
    Eigen::Matrix3d matrix_33d; // Eigen::Matrix<double, 3, 3> matrix_33d
    // ！！！错误：最多到Eigen::Matrix4d
    // Eigen::Matrix5d matrix_55d = Eigen::Matrix5d::Zero();

    // 动态大小：如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic_d;
    Eigen::MatrixXd matrix_x_d;

    // 初始化
    Eigen::Matrix4d matrix_44d = Eigen::Matrix4d::Zero();   // 初始化为零
    std::cout << matrix_44d << std::endl;
    Eigen::Matrix2d matrix_22d;
    matrix_22d << 0, 1, 2, 3;   // 输入数据进行初始化
    std::cout << matrix_22d << std::endl;

    // 访问矩阵中元素
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            std::cout << matrix_22d(i, j) << " ";
        }
        std::cout << std::endl;
    }

    // 矩阵和向量相乘
    // ！！！错误：在Eigen中，不能两种不同类型进行运算
    // Eigen::Matrix<float, 2, 3> matrix_23f;
    // matrix_23f << 0.0, 1.1, 2.2, 3.3, 4.4, 5.5;
    // Eigen::Matrix<double, 3, 1> vector_3d;
    // matrix_23d << 4.4, 5.5, 6.6;
    // Eigen::Matrix<double, 2, 1> result = matrix_23f * vector_3d;

    // 如果一定要不同类型计算，则需要显示转换
    matrix_23f << 0.0, 1.1, 2.2, 3.3, 4.4, 5.5;
    vector_3d  << 6.6, 7.7, 8.8;
    Eigen::Matrix<double, 2, 1> result = matrix_23f.cast<double>() * vector_3d;
    std::cout << result << std::endl;

    // 其他常见运算（四则运算用+-*/即可）
    matrix_33d = Eigen::Matrix3d::Random();                     // 随机数矩阵
    std::cout << "random: " << matrix_33d << std::endl;
    Eigen::Matrix3d matrix_33d_trans = matrix_33d.transpose();  // 矩阵转置
    std::cout << "transpose: " << matrix_33d_trans << std::endl;
    Eigen::Matrix3d matrix_inverse = matrix_33d.inverse();      // 矩阵的逆
    std::cout << "inverse: " << matrix_inverse << std::endl;
    double matrix_sum = matrix_33d.sum();                       // 矩阵各元素和
    std::cout << "sum: " << matrix_sum << std::endl;
    double matrix_trace = matrix_33d.trace();                   // 矩阵的迹
    std::cout << "trace: " <<  matrix_trace << std::endl;
    double matrix_det = matrix_33d.determinant();               // 矩阵行列式
    std::cout << "det: " << matrix_det << std::endl;

    // 特征值和特征向量
    // Eigen::SelfAdjointEigenSolver用于计算对称矩阵（或自伴矩阵）的特征值和特征向量
    // 实对称矩阵可以保证对角化成功
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33d.transpose() * matrix_33d);
    std::cout << "Eigen values = " << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors = " << eigen_solver.eigenvectors() << std::endl;

    // 解方程 matrix_NN * x = v_Nd
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();  // 保证半正定
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock(); // 计时

    // 直接求逆：最直接的，但是求逆运算量大
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "x = " << x.transpose() << std::endl;
    std::cout << "time of normal inverse is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;

    time_stt = clock();
    // 矩阵分解：例如QR分解，速度会快很多
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    std::cout << "x = " << x.transpose() << std::endl;
    std::cout << "time of Qr decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;

    time_stt = clock();
    // 对于正定矩阵，还可以用cholesky分解来解方程
    x = matrix_NN.ldlt().solve(v_Nd);
    std::cout << "x = " << x.transpose() << std::endl;
    std::cout << "time of ldlt decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;

    return 0;
}