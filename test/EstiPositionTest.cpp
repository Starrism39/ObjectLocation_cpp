#include <vector>
#include <iostream>
#include "modules/esti_position.h"  // 包含EstiPosition类的头文件


void initBBoxParameters(Package& pkg, int img_width=1920, int img_height=1080) {
    // 原始BBox参数 [x,y,w,h] (像素坐标)
    pkg.Bbox = {
        static_cast<int>(pkg.norm_Bbox[0] * img_width),   // x
        static_cast<int>(pkg.norm_Bbox[1] * img_height),  // y
        static_cast<int>(pkg.norm_Bbox[2] * img_width),   // width
        static_cast<int>(pkg.norm_Bbox[3] * img_height)   // height
    };

    // 验证BBox有效性
    if (pkg.Bbox[0] < 0 || pkg.Bbox[1] < 0 || 
        pkg.Bbox[0]+pkg.Bbox[2] > img_width ||
        pkg.Bbox[1]+pkg.Bbox[3] > img_height) {
        throw std::runtime_error("Invalid BBox coordinates");
    }
}

// 模拟BBox中心转换的辅助函数
std::vector<double> simulateBBoxProjection(const Package& pkg) {
    // 这里需要实现实际算法，此处为示例伪代码
    // 步骤1: 从归一化BBox获取中心点
    double cx = pkg.norm_Bbox[0] + pkg.norm_Bbox[2]/2;
    double cy = pkg.norm_Bbox[1] + pkg.norm_Bbox[3]/2;
    
    // 步骤2: 使用相机参数反投影到3D空间
    // （此处应包含完整的相机模型计算，示例使用简单线性变换）
    double world_x = cx * pkg.camera_K[0] + pkg.camera_K[2];
    double world_y = cy * pkg.camera_K[1] + pkg.camera_K[3];
    
    // 步骤3: 坐标系转换到UTM（示例简单转换）
    return {
        pkg.camera_pose[3] + world_x * 0.1,  // X坐标转换
        pkg.camera_pose[4] + world_y * 0.1,  // Y坐标转换
        pkg.camera_pose[5] - 50.0            // 假设高度下降50米
    };
}

int main() {
    // ---------------------- 测试数据初始化 ----------------------
    // 创建测试用的Package对象
    Package test_pkg;
    
    // 填充相机参数 (示例值，需根据实际数据调整)
    test_pkg.camera_pose = {1.57, 0.0, 0.0, 500.0, 600.0, 200.0};  // [yaw, pitch, roll, x, y, z]
    test_pkg.camera_K = {1000.0, 1000.0, 320.0, 240.0};           // [fx, fy, cx, cy]
    test_pkg.camera_distortion = {0.01, -0.02, 0.001, 0.002, 0.005};     // [k1, k2, p1, p2, k3]
    test_pkg.norm_Bbox = {0.5, 0.5, 0.2, 0.3};                   // 归一化BBox
    initBBoxParameters(test_pkg);
    test_pkg.uav_utm = simulateBBoxProjection(test_pkg);                    // 无人机UTM坐标

    // ---------------------- 场景1：启用相机位姿定位 ----------------------
    {
        std::cout << "\n===== 测试场景1：enable=true (相机位姿定位) =====" << std::endl;
        
        // 初始化估计器（单尺度地图模式）
        EstiPosition estimator(
            false,                              // is_multi_map
            "/home/xjy/code/location_Map/test_1/data/mesh_triangles.npy",                // 替换为实际网格路径
            60.0,                               // default_height
            "szyx",                             // 欧拉角顺序
            true                                // enable=true
        );
        
        // 处理数据
        estimator.process(test_pkg);
        
        // 输出结果
        std::cout << "目标位置 (UTM): ";
        for (auto coord : test_pkg.location) {
            std::cout << coord << " ";
        }
        std::cout << std::endl;
    }

    // ---------------------- 场景2：使用无人机目标点定位 ----------------------
    {
        std::cout << "\n===== 测试场景2：enable=false (无人机目标点定位) =====" << std::endl;
        
        // 初始化估计器（关闭相机定位）
        EstiPosition estimator(
            false,                              // is_multi_map
            "/home/xjy/code/location_Map/test_1/data/mesh_triangles.npy",                // 替换为实际网格路径
            60.0,                               // default_height
            "rzyx",                             // 欧拉角顺序
            false                               // enable=false
        );
        
        // 处理数据
        estimator.process(test_pkg);
        
        // 输出结果
        std::cout << "目标位置 (UTM): ";
        for (auto coord : test_pkg.location) {
            std::cout << coord << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
