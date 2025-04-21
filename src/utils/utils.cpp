#include "utils/utils.h"
#include <cmath>


CameraPose setCameraPose(const std::vector<double> &camera_pose, const std::string &order)
{
    // 参数检查
    if (camera_pose.size() != 6)
    {
        throw std::invalid_argument("相机姿态参数必须包含6个值");
    }

    // 构建平移向量
    Eigen::Vector3d t(camera_pose[3], camera_pose[4], camera_pose[5]);

    // 计算旋转矩阵
    Eigen::Matrix3d R = euler2mat(camera_pose[0], camera_pose[1], camera_pose[2], order);

    // 计算旋转矩阵的逆
    Eigen::Matrix3d R_inv = R.inverse();

    return {R, t, R_inv};
}

std::vector<double> setDistortionCoeffs(const std::vector<double> &distortion_param)
{
    // 参数检查
    if (distortion_param.size() != 5)
    {
        throw std::invalid_argument("畸变参数必须包含5个值");
    }

    // 获取各个畸变参数
    double k1 = distortion_param[0]; // 径向畸变系数1
    double k2 = distortion_param[1]; // 径向畸变系数2
    double p1 = distortion_param[2]; // 切向畸变系数1
    double p2 = distortion_param[3]; // 切向畸变系数2
    double k3 = distortion_param[4]; // 径向畸变系数3

    // 返回畸变系数数组
    return {k1, k2, p1, p2, k3};
}

CameraMatrixUtils setK(const std::vector<double> &cam_K)
{
    // 从输入向量中获取相机参数
    double fx = cam_K[0]; // 焦距x
    double fy = cam_K[1]; // 焦距y
    double cx = cam_K[2]; // 主点x坐标
    double cy = cam_K[3]; // 主点y坐标

    // 构建内参矩阵
    Eigen::Matrix3d K;
    K << fx, 0, cx,
        0, fy, cy,
        0, 0, 1;

    // 计算内参矩阵的逆
    Eigen::Matrix3d K_inv = K.inverse();

    // 返回结果
    return {K, K_inv};
}

Eigen::Vector3d getRay(const std::vector<double> &pixel,
                       const Eigen::Matrix3d &K_inv,
                       const std::vector<double> &distortion_coeffs,
                       const Eigen::Matrix3d &rotation)
{
    /*
    获取从相机像素点出发的射线方向。

    Args:
        pixel: 相机像素点坐标。
        K_inv: 相机内参的逆矩阵。
        distortion_coeffs: 畸变系数，形状为 (N,) 的浮点型数组，N 是畸变系数的数量。
        rotation: 相机旋转矩阵，形状为 (3, 3) 的浮点型数组。

    Returns:
        ray: 射线方向向量，形状为 (3,) 的浮点型数组，表示从相机像素点出发的单位方向向量。
    */

    // 将像素坐标转换为齐次坐标
    Eigen::Vector3d pixel_homogeneous;
    pixel_homogeneous << pixel[0], pixel[1], 1.0;

    // 计算相机坐标系下的点
    Eigen::Vector3d p_cam = K_inv * pixel_homogeneous;
    std::cout << "p_cam: " << p_cam[0] << " " << p_cam[1] << " " << p_cam[2] << std::endl;

    // 畸变校正
    p_cam = undistort_pixel_coords(p_cam, distortion_coeffs);
    p_cam = {0, 0, 1};
    
    Eigen::Vector3d p_cam_changed = {p_cam[2], -p_cam[0], p_cam[1]};

    // 转换到世界坐标系
    Eigen::Vector3d p_world = rotation * p_cam_changed;
    p_world = p_world.normalized();

    // 归一化为单位向量
    return {p_world[1], p_world[0], -p_world[2]};
}

Eigen::Vector3d undistort_pixel_coords(const Eigen::Vector3d &p_cam,
                                       const std::vector<double> &distortion_coeffs)
{
    // 提取归一化坐标的x和y分量
    double x = p_cam.x();
    double y = p_cam.y();

    // 初始化为输入的畸变坐标
    double x_undist = x;
    double y_undist = y;

    // 迭代求解去畸变坐标（通常5次迭代足够）
    const int num_iterations = 5;
    for (int i = 0; i < num_iterations; ++i)
    {
        double r2 = x_undist * x_undist + y_undist * y_undist;
        double r4 = r2 * r2;
        double r6 = r4 * r2;

        // 计算径向畸变系数
        double radial = 1.0;
        if (distortion_coeffs.size() > 0)
        {
            radial += distortion_coeffs[0] * r2;
        }
        if (distortion_coeffs.size() > 1)
        {
            radial += distortion_coeffs[1] * r4;
        }
        if (distortion_coeffs.size() > 4)
        {
            radial += distortion_coeffs[4] * r6;
        }

        // 计算切向畸变
        double tangential_x = 0.0;
        double tangential_y = 0.0;
        if (distortion_coeffs.size() > 2)
        {
            double p1 = distortion_coeffs[2];
            double p2 = distortion_coeffs[3];
            tangential_x = 2 * p1 * x_undist * y_undist + p2 * (r2 + 2 * x_undist * x_undist);
            tangential_y = p1 * (r2 + 2 * y_undist * y_undist) + 2 * p2 * x_undist * y_undist;
        }

        // 计算当前猜测的畸变坐标
        double x_distorted = x_undist * radial + tangential_x;
        double y_distorted = y_undist * radial + tangential_y;

        // 更新去畸变坐标的估计值
        x_undist += x - x_distorted;
        y_undist += y - y_distorted;
    }

    // 返回去畸变后的归一化坐标（齐次形式）
    return Eigen::Vector3d(x_undist, y_undist, 1.0);
}

Eigen::Matrix3d euler2mat(double yaw, double pitch, double roll, const std::string &order)
{
    // 转换为弧度
    double y = yaw * M_PI / 180.0;
    double p = pitch * M_PI / 180.0;
    double r = roll * M_PI / 180.0;

    // 计算各个轴的旋转矩阵
    Eigen::Matrix3d Rx, Ry, Rz;

    Rx << 1, 0, 0,
        0, cos(r), -sin(r),
        0, sin(r), cos(r);

    Ry << cos(p), 0, sin(p),
        0, 1, 0,
        -sin(p), 0, cos(p);

    Rz << cos(y), -sin(y), 0,
        sin(y), cos(y), 0,
        0, 0, 1;

    // 根据旋转顺序组合
    if (order == "szxy")
    {
        return Rz * Rx * Ry;
    }
    else if (order == "sxyz")
    {
        return Rx * Ry * Rz;
    }
    else if (order == "syzx")
    {
        return Ry * Rz * Rx;
    }
    else if (order == "sxzy")
    {
        return Rx * Rz * Ry;
    }
    else if (order == "syxz")
    {
        return Ry * Rx * Rz;
    }
    else if (order == "szyx")
    {
        return Rz * Ry * Rx;
    }
    throw std::invalid_argument("不支持的旋转顺序");
}
