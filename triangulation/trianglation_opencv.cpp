#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 相机内参矩阵 K (fx, fy, cx, cy)
    cv::Mat K = (cv::Mat_<double>(3, 3) << 800, 0, 320, 
                                           0, 800, 240, 
                                           0, 0, 1);

    // 相机外参矩阵 (旋转 R 和 平移 t)
    std::vector<cv::Mat> R = { (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                                            0, 1, 0,
                                                            0, 0, 1),
                               (cv::Mat_<double>(3, 3) << 0.99, 0.01, 0,
                                                            -0.01, 0.99, 0,
                                                             0, 0, 1) }; // 假设两帧图像旋转略有差异
    std::vector<cv::Mat> t = { (cv::Mat_<double>(3, 1) << 0, 0, 0),
                               (cv::Mat_<double>(3, 1) << 0.5, 0, 0) }; // 假设平移向量

    // 两帧图像中目标点的像素坐标 (u, v)
    std::vector<cv::Point2d> points_img1 = { cv::Point2d(350, 250) }; // 图像1中的像素坐标
    std::vector<cv::Point2d> points_img2 = { cv::Point2d(355, 255) }; // 图像2中的像素坐标

    // 将像素坐标转换为归一化相机坐标
    std::vector<cv::Mat> proj_matrices;
    for (size_t i = 0; i < R.size(); ++i) {
        // 构建投影矩阵 P = K * [R|t]
        cv::Mat Rt;
        cv::hconcat(R[i], t[i], Rt);  // [R|t]
        cv::Mat P = K * Rt;           // K * [R|t]
        proj_matrices.push_back(P);
    }

    // 构建齐次像素坐标矩阵
    cv::Mat pts1 = (cv::Mat_<double>(2, 1) << points_img1[0].x, points_img1[0].y);
    cv::Mat pts2 = (cv::Mat_<double>(2, 1) << points_img2[0].x, points_img2[0].y);

    // 进行三角化
    cv::Mat points_4d;
    cv::triangulatePoints(proj_matrices[0], proj_matrices[1], pts1, pts2, points_4d);

    // 将齐次坐标转换为3D坐标
    cv::Mat point_3d = points_4d.col(0);
    point_3d /= point_3d.at<double>(3);  // 归一化

    std::cout << "3D Point: " << point_3d.t() << std::endl;

    return 0;
}
