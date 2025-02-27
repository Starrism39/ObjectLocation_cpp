#include <Eigen/Dense>
#include "utils/npy_reader.h"  // ReadMesh

struct CameraMatrix {
    Eigen::Matrix3d K;      // 相机内参矩阵
    Eigen::Matrix3d K_inv;  // 相机内参矩阵的逆
};

struct CameraPose {
    Eigen::Matrix3d R;        // 旋转矩阵
    Eigen::Vector3d t;        // 平移向量
    Eigen::Matrix3d R_inv;    // 旋转矩阵的逆
};

Eigen::Vector3d undistort_pixel_coords(const Eigen::Vector3d& p_cam, const std::vector<double>& distortion_coeffs);  // 对相机坐标进行畸变校正
Eigen::Matrix3d euler2mat(double yaw, double pitch, double roll, const std::string& order = "szxy");  // 欧拉角转旋转矩阵

CameraMatrix setK(const std::vector<double>& cam_K);  // 从输入向量中获取相机参数
Eigen::Vector3d getRay(const std::vector<double>& pixel,
                    const Eigen::Matrix3d& K_inv,
                    const std::vector<double>& distortion_coeffs,
                    const Eigen::Matrix3d& rotation);  // 获取从相机像素点出发的射线方向
std::vector<double> setDistortionCoeffs(const std::vector<double>& distortion_param);  // 设置畸变系数
CameraPose setCameraPose(const std::vector<double>& camera_pose, const std::string& order = "szxy");  // 设置相机姿态
