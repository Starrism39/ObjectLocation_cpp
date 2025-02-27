#include <vector>
#include <stdexcept>
#include <iostream>
#include "utils/npy_reader.h"
#include "utils/mesh_raycast.h"


int main() {

    // 读取npy文件中的三角形网格数据
    const std::string filename = "/home/xjy/code/location_Map/test_1/data/mesh_triangles.npy";
    std::vector<std::vector<std::vector<float>>> mesh_data;
    size_t num_triangles, vertices_per_triangle, coords_per_vertex;
    if (!NPYReader::ReadMesh(filename, mesh_data, 
                                    num_triangles, vertices_per_triangle, coords_per_vertex)) {
        std::cerr << "Failed to read mesh data" << std::endl;
        return -1;
    }

    // 设置光线参数
    glm::vec3 source(0.0f, 0.0f, 210.0f);        // 从 z=210 向下发射光线
    glm::vec3 direction(0.0f, 0.0f, -1.0f);     // 向下的方向

    // 射线投射
    RaycastHit results;
    if(estiPosition(source, direction, mesh_data, num_triangles, results)){
        std::cout << "找到目标位置：" << std::endl;
        std::cout << "  面: " << results.face << std::endl;
        std::cout << "  点: " << results.point.x << ", " << results.point.y << ", " << results.point.z << std::endl;
        std::cout << "  法线: " << results.normal.x << ", " << results.normal.y << ", " << results.normal.z << std::endl;
        std::cout << "  重心坐标: " << results.coeff.x << ", " << results.coeff.y << std::endl;
        std::cout << "  距离: " << results.distance << std::endl;
        std::cout << "  点积: " << results.dot << std::endl;
    }

    return 0;
}
