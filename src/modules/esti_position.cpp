#include <iostream>
#include <algorithm>

#include "modules/esti_position.h"

void printMeshData(const std::vector<std::vector<std::vector<float>>> &mesh_data,
                   int max_layers = 10, // 最大打印层数（防止数据过大）
                   int max_rows = 5,   // 每层最大行数
                   int max_cols = 5)
{ // 每行最大列数
    std::cout << "Mesh Data ["
              << mesh_data.size() << " layers]"
              << std::endl;

    // 遍历每一层
    for (size_t layer = 0; layer < mesh_data.size() && layer < max_layers; ++layer)
    {
        std::cout << "\n── Layer " << layer << " ──\n";

        // 遍历每一行
        for (size_t row = 0; row < mesh_data[layer].size() && row < max_rows; ++row)
        {
            std::cout << "  Row " << row << ": [ ";

            // 遍历每个元素
            for (size_t col = 0; col < mesh_data[layer][row].size() && col < max_cols; ++col)
            {
                std::cout << mesh_data[layer][row][col];
                if (col != mesh_data[layer][row].size() - 1)
                    std::cout << ", ";
            }

            // 如果数据被截断，显示省略号
            if (mesh_data[layer][row].size() > max_cols)
                std::cout << ", ...";
            std::cout << " ]\n";
        }

        // 如果行数被截断，显示省略信息
        if (mesh_data[layer].size() > max_rows)
        {
            std::cout << "  ... (showing " << max_rows
                      << "/" << mesh_data[layer].size()
                      << " rows)\n";
        }
    }

    // 如果层数被截断，显示省略信息
    if (mesh_data.size() > max_layers)
    {
        std::cout << "\n... (showing " << max_layers
                  << "/" << mesh_data.size()
                  << " layers)\n";
    }
}

EstiPosition::EstiPosition(bool is_multi_map,
                           const std::string &mesh_path,
                           double default_height,
                           const std::string &order,
                           bool enable,
                           int max_queue_length,
                           int multi_num,
                           const std::vector<double> &downSampled_scale,
                           const std::vector<std::vector<double>> &split_scale,
                           int block_num,
                           double overlap_x_scale,
                           double overlap_y_scale)
    : Location::Location("EstiPosition", max_queue_length),
      is_multi_map(is_multi_map),
      order(order),
      default_height(default_height),
      enable(enable)
{
    // multi_mesh(nullptr) {

    if (is_multi_map)
    {
        // TODO: 稍后实现多尺度地图功能
        std::cout << "多尺度地图功能将在稍后实现。" << std::endl;
        // multi_mesh = new MultiMesh(...); // 未来实现
    }
    else
    {
        // 加载单尺度网格
        if (!mesh_path.empty())
        {
            if (!loadMesh(mesh_path))
            {
                std::cerr << "无法从以下路径加载网格: " << mesh_path << std::endl;
            }
        }
    }
}

EstiPosition::~EstiPosition()
{
    if (is_multi_map)
    {
        // TODO: 在实现时清理多网格资源
    }
}

bool EstiPosition::loadMesh(const std::string &mesh_path)
{
    return NPYReader::ReadMesh(mesh_path, mesh_data, num_triangles, vertices_per_triangle, coords_per_vertex);
}

std::vector<double> EstiPosition::getPoint(const Package &data)
{
    // 设置相机参数
    auto camera_matrix = setK(data.camera_K);
    Eigen::Matrix3d K_inv = camera_matrix.K_inv;

    std::vector<double> D = setDistortionCoeffs(data.camera_distortion);

    // 设置相机姿态
    auto pose = setCameraPose(data.camera_pose, order);
    Eigen::Matrix3d R = pose.R;
    Eigen::Vector3d t = pose.t;

    // 从像素点获取射线
    Eigen::Vector3d ray = getRay(data.getCenterPoint(), K_inv, D, R);

    if (is_multi_map)
    {
        // TODO: 实现多尺度地图处理
        std::cout << "多尺度地图射线投射将在稍后实现。" << std::endl;

        // 暂时使用默认回退方案
        double l = (default_height - t.z()) / -ray.z();
        Eigen::Vector3d inter_point = t - l * ray;

        return {inter_point.x(), inter_point.y(), inter_point.z()};
    }
    else
    {
        // 单尺度地图实现
        glm::vec3 source(t.x(), t.y(), t.z());
        glm::vec3 direction(ray.x(), ray.y(), ray.z());
        // std::cout << "t: " << t[0] << " " << t[1] << " " << t[2] << std::endl;
        // std::cout << "ray: " << ray[0] << " " << ray[1] << " " << ray[2] << std::endl;

        std::vector<RaycastHit> result = raycast(source, direction, mesh_data, num_triangles);
        // std::cout << "num_triangles: " << num_triangles << std::endl;
        // printMeshData(mesh_data);

        if (result.empty())
        {
            // 无交点，使用默认深度
            // double l = (default_height - t.z()) / -ray.z();
            // Eigen::Vector3d inter_point = t - l * ray;
            Eigen::Vector3d inter_point = t - data.dp->get_laser_distance() * ray;

            return {inter_point.x(), inter_point.y(), inter_point.z()};
        }
        else
        {
            // 查找最近的交点
            auto closest_hit = *std::min_element(result.begin(), result.end(),
                                                 [](const RaycastHit &a, const RaycastHit &b)
                                                 {
                                                     return a.distance < b.distance;
                                                 });

            return {closest_hit.point.x, closest_hit.point.y, closest_hit.point.z};
        }
    }
}

std::vector<double> EstiPosition::getPointFormUavObjectPoint(const Package &data)
{
    // 相机位置
    Eigen::Vector3d p_camera(data.camera_pose[3], data.camera_pose[4], data.camera_pose[5]);

    // UTM坐标中的目标位置
    Eigen::Vector3d p_obj(data.uav_utm[0], data.uav_utm[1], data.uav_utm[2]);

    // 从相机到目标的方向
    Eigen::Vector3d ray = p_obj - p_camera;
    ray.normalize();

    if (is_multi_map)
    {
        // TODO: 实现多尺度地图处理
        std::cout << "从无人机目标点进行多尺度地图射线投射将在稍后实现。" << std::endl;

        // 默认回退方案
        double l = (default_height - p_camera.z()) / -ray.z();
        Eigen::Vector3d inter_point = p_camera - l * ray;

        return {inter_point.x(), inter_point.y(), inter_point.z()};
    }
    else
    {
        // 单尺度地图实现
        glm::vec3 source(p_camera.x(), p_camera.y(), p_camera.z());
        glm::vec3 direction(ray.x(), ray.y(), ray.z());

        std::vector<RaycastHit> result = raycast(source, direction, mesh_data, num_triangles);

        if (result.empty())
        {
            // 无交点，使用默认深度
            // double l = (default_height - p_camera.z()) / -ray.z();
            // Eigen::Vector3d inter_point = p_camera - l * ray;
            Eigen::Vector3d inter_point = p_camera - data.dp->get_laser_distance() * ray;

            return {inter_point.x(), inter_point.y(), inter_point.z()};
        }
        else
        {
            // 查找最近的交点
            auto closest_hit = *std::min_element(result.begin(), result.end(),
                                                 [](const RaycastHit &a, const RaycastHit &b)
                                                 {
                                                     return a.distance < b.distance;
                                                 });

            return {closest_hit.point.x, closest_hit.point.y, closest_hit.point.z};
        }
    }
}

void EstiPosition::process(Package &data)
{
    if (enable)
    {
        data.location = getPoint(data);
    }
    else
    {
        data.location = getPointFormUavObjectPoint(data);
    }

    // // 检查位置是否在默认高度，如果是则清除目标向量
    // if (data.location.size() >= 3 && std::abs(data.location[2] - default_height) < 1e-6)
    // {
    //     data.location = data.uav_utm;
    //     // data.location.clear();
    // }

    // 极端值的调试输出
    if (data.location.size() >= 1 && std::abs(data.location[0]) > 1000)
    {
        std::cout << "data.camera_pose: "
                  << data.camera_pose[3] << ", "
                  << data.camera_pose[4] << ", "
                  << data.camera_pose[5] << std::endl;
        std::cout << "data.uav_utm: "
                  << data.uav_utm[0] << ", "
                  << data.uav_utm[1] << ", "
                  << data.uav_utm[2] << std::endl;
        std::cout << "data.location: "
                  << data.location[0] << ", "
                  << data.location[1] << ", "
                  << data.location[2] << std::endl;
        std::cout << "data.time: " << data.time << std::endl;
    }

    // 最终回退方案 - 始终使用无人机UTM坐标
    // data.location = data.uav_utm;
}