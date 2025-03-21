#include "utils/mesh_raycast.h"
#include <limits>
std::vector<RaycastHit> raycast(const glm::vec3 &source, const glm::vec3 &direction,
                                const std::vector<std::vector<std::vector<float>>> &mesh_data, int num_triangles)
{

    if (num_triangles <= 0)
    {
        throw std::invalid_argument("三角形数量必须为正数");
    }

    glm::vec3 d = -glm::normalize(direction); // 取反并归一化方向向量
    std::vector<RaycastHit> hits;

    for (int i = 0; i < num_triangles; ++i)
    {
        // 使用新的 vertex 函数直接从 mesh_data 获取顶点
        const glm::vec3 a = vertex(mesh_data, i, 0);
        const glm::vec3 b = vertex(mesh_data, i, 1) - a; // b 和 c 相对于 a
        const glm::vec3 c = vertex(mesh_data, i, 2) - a;
        const glm::vec3 g = source - a; // g 相对于 a

        const float det = glm::determinant(glm::mat3(b, c, d));
        if (std::abs(det) < 1e-6f)
        {             // 检查行列式是否接近零 (平行或退化)
            continue; // 跳过这个三角形
        }

        // 计算重心坐标 (n, m) 和距离 (k)
        const float n = glm::determinant(glm::mat3(g, c, d)) / det;
        const float m = glm::determinant(glm::mat3(b, g, d)) / det;
        const float k = glm::determinant(glm::mat3(b, c, g)) / det;

        // 检查交点是否在三角形内且在光线原点的前面
        if (n >= 0.0f && m >= 0.0f && n + m <= 1.0f && k >= 0.0f)
        {
            const glm::vec3 pt = source + direction * k;             // 交点
            const glm::vec3 norm = glm::normalize(glm::cross(b, c)); // 三角形法线
            const float dot = glm::dot(d, norm);                     // 取反方向向量和法线的点积

            hits.emplace_back(RaycastHit{i, pt, norm, glm::vec2(n, m), k, dot}); // 存储命中信息
        }
    }

    return hits;
}

bool estiPosition(const glm::vec3 &source, const glm::vec3 &direction,
                  const std::vector<std::vector<std::vector<float>>> &mesh_data, int num_triangles, RaycastHit &point)
{

    float min_distance = std::numeric_limits<float>::max();
    try
    {
        // 调用 raycast，直接传递 mesh_data
        std::vector<RaycastHit> results = raycast(source, direction, mesh_data, num_triangles);

        // 处理结果
        if (results.empty())
        {
            std::cout << "未找到交点。" << std::endl;
            return false;
        }
        else
        {
            for (const auto &hit : results)
            {
                // std::cout << "找到交点：" << std::endl;
                // std::cout << "  面: " << hit.face << std::endl;
                // std::cout << "  点: " << hit.point.x << ", " << hit.point.y << ", " << hit.point.z << std::endl;
                // std::cout << "  法线: " << hit.normal.x << ", " << hit.normal.y << ", " << hit.normal.z << std::endl;
                // std::cout << "  重心坐标: " << hit.coeff.x << ", " << hit.coeff.y << std::endl;
                // std::cout << "  距离: " << hit.distance << std::endl;
                // std::cout << "  点积: " << hit.dot << std::endl;
                if (min_distance > hit.distance)
                {
                    min_distance = hit.distance;
                    point = hit;
                }
            }
        }
    }
    catch (const std::invalid_argument &e)
    {
        std::cerr << "错误: " << e.what() << std::endl;
        return false;
    }
    return true;
}