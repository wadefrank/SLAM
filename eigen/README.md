# Eigen

## Version
Version: 3.3.9  
GitLab: https://gitlab.com/libeigen/eigen/-/releases/3.3.9  

## Build
```bash
unzip eigen-3.3.9.zip
cd eigen-3.3.9/
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/wade/third_party/eigen-3.3.9/install ..
make -j4
make install
```

## include

### Eigen/Dense
```C++
#include <Eigen/Dense>
```
- 功能：包含 Eigen 的所有主要功能，适合大部分线性代数需求。
- 常用功能：
  - 矩阵和向量操作（Eigen::Matrix、Eigen::Vector）
  - 矩阵乘法、转置、逆等基础运算
  - 基础线性方程求解
  - 矩阵分解（LU、QR 等）
- 使用场景：最常见的头文件，如果你需要 Eigen 的通用功能，可以直接包含这个文件。

### Eigen/Core
```C++
#include <Eigen/Core>
```
- 功能：包含 Eigen 的核心数据类型和基本功能。
- 常用功能：
  - 矩阵和向量类的定义
  - 基本的矩阵运算（加法、减法、乘法、标量操作等）
- 使用场景：当你只需要基本的矩阵操作，且不需要其他高级功能时，使用 Eigen/Core 可以加快编译速度。