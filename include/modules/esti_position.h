#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "framework/module.h"
#include "utils/mesh_raycast.h"
#include "utils/npy_reader.h"
#include "utils/utils.h"
// #include "utils/mesh_raycast.h"  // TODO

class EstiPosition {
public:
    EstiPosition(bool is_multi_map = false,
                const std::string& mesh_pash = "",
                double default_height = 60,
                const std::string&  order = "rzyx",
                bool enable = true,
                int max_queue_length = -1,
                int multi_num = 2,
                const std::vector<double>& downSampled_scale = {0.2, 0.1},
                const std::vector<std::vector<double>>& split_scale = {{10, 10}, {5, 5}},
                int block_num = 1,
                double overlap_x_scale = 0.1,
                double overlap_y_scale = 0.1);
    ~EstiPosition();

    std::vector<double> getPoint(const Package& data);
    std::vector<double> getPointFormUavObjectPoint(const Package& data);
    void process(Package& data);

private:
    bool is_multi_map;
    std::string order;
    double default_height;
    bool enable;  // true:根据相机位姿定位目标点 false:根据道通uav_obj_pose重定位目标点

    // 单尺度地图
    std::vector<std::vector<std::vector<float>>> mesh_data;
    size_t num_triangles;
    size_t vertices_per_triangle;
    size_t coords_per_vertex;

    // 多尺度地图
    // TODO

    bool loadMesh(const std::string& mesh_path);
};