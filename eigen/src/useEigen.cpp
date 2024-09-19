#include <iostream>

#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

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
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < 2; j++){
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
    std::cout << matrix_33d << std::endl;
    Eigen::Matrix3d matrix_33d_trans = matrix_33d.transpose();  // 矩阵转置
    std::cout << matrix_33d_trans << std::endl;
    double matrix_sum = matrix_33d.sum();                       // 矩阵各元素和
    std::cout << matrix_sum << std::endl;

    return 0;
}