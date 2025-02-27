#include <vector>
#include <stdexcept>
#include <iostream>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_access.hpp" // For glm::determinant

inline glm::vec3 vertex(const std::vector<std::vector<std::vector<float>>>& mesh_data, int triangle_index, int vertex_index) {
    return glm::vec3(
        mesh_data[triangle_index][vertex_index][0],
        mesh_data[triangle_index][vertex_index][1],
        mesh_data[triangle_index][vertex_index][2]
    );
}

// 定义一个结构体来存储相交结果
struct RaycastHit {
    int face;
    glm::vec3 point;
    glm::vec3 normal;
    glm::vec2 coeff; // 使用 glm::vec2 存储重心坐标 n 和 m
    float distance;
    float dot;
};


std::vector<RaycastHit> raycast(const glm::vec3& source, const glm::vec3& direction,
                                 const std::vector<std::vector<std::vector<float>>>& mesh_data, int num_triangles);

bool estiPosition(const glm::vec3& source, const glm::vec3& direction,
                                 const std::vector<std::vector<std::vector<float>>>& mesh_data, int num_triangles, RaycastHit& point);