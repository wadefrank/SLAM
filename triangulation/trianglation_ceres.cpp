#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>

// 定义相机模型残差
struct ReprojectionError {
    ReprojectionError(double observed_x, double observed_y, 
                      const cv::Mat& K, const cv::Mat& R, const cv::Mat& t)
        : observed_x(observed_x), observed_y(observed_y), K(K), R(R), t(t) {}

    template <typename T>
    bool operator()(const T* const point_3d, T* residuals) const {
        // 将3D点从世界坐标系转换到相机坐标系
        T p[3];
        T cam_R[9], cam_t[3];
        for (int i = 0; i < 9; ++i) cam_R[i] = T(R.at<double>(i / 3, i % 3));
        for (int i = 0; i < 3; ++i) cam_t[i] = T(t.at<double>(i));

        // 相机旋转和平移矩阵作用到3D点
        ceres::AngleAxisRotatePoint(cam_R, point_3d, p);
        p[0] += cam_t[0];
        p[1] += cam_t[1];
        p[2] += cam_t[2];

        // 将点从相机坐标系投影到归一化平面
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        // 将归一化坐标映射到像素坐标
        T predicted_x = T(K.at<double>(0, 0)) * xp + T(K.at<double>(0, 2));
        T predicted_y = T(K.at<double>(1, 1)) * yp + T(K.at<double>(1, 2));

        // 计算残差（重投影误差）
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);

        return true;
    }

    double observed_x;
    double observed_y;
    cv::Mat K;  // 相机内参矩阵
    cv::Mat R;  // 相机旋转矩阵
    cv::Mat t;  // 相机位移向量
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 3D点初始值猜测（假设位于原点）
    double point_3d[3] = {0.0, 0.0, 0.0};

    // 设置相机内参矩阵 (fx, fy, cx, cy)
    cv::Mat K = (cv::Mat_<double>(3, 3) << 800, 0, 320, 
                                           0, 800, 240, 
                                           0, 0, 1);

    // 设置每帧图像的位姿（旋转矩阵和位移向量）
    std::vector<cv::Mat> rotations = { ... };  // 假设已经获取到旋转矩阵列表
    std::vector<cv::Mat> translations = { ... };  // 假设已经获取到位移向量列表

    // 每帧图像中观测到的像素坐标
    std::vector<cv::Point2d> observed_points = { ... };  // 假设已经获取到像素坐标

    // 构建 Ceres 问题
    ceres::Problem problem;

    // 为每帧图像的观测值添加残差
    for (size_t i = 0; i < observed_points.size(); ++i) {
        ceres::CostFunction* cost_function = 
            new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(
                new ReprojectionError(observed_points[i].x, observed_points[i].y, 
                                      K, rotations[i], translations[i]));
        
        problem.AddResidualBlock(cost_function, nullptr, point_3d);
    }

    // 设置优化选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // 运行优化
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 输出优化结果
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "Final 3D point: [" 
              << point_3d[0] << ", " 
              << point_3d[1] << ", " 
              << point_3d[2] << "]" << std::endl;

    return 0;
}
