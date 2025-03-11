#include "modules/spatial_filter.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <ctime>

// 用于生成随机数的辅助函数
class RandomGenerator {
private:
    std::random_device rd;
    std::mt19937 gen;
    
public:
    RandomGenerator() : gen(rd()) {}
    
    // 生成指定范围内的随机浮点数
    double generateFloat(double min, double max) {
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }
    
    // 生成指定范围内的随机整数
    int generateInt(int min, int max) {
        std::uniform_int_distribution<> dis(min, max);
        return dis(gen);
    }
};

// 生成模拟的Package数据
std::vector<Package> generateSimulatedData(RandomGenerator& rng, int num_targets, int num_uavs) {
    std::vector<Package> packages;
    time_t current_time = std::time(nullptr);
    
    // 为每个目标生成多个观测数据
    for(int target = 0; target < num_targets; target++) {
        // 随机生成目标的真实位置（WGS84坐标）
        double base_lat = rng.generateFloat(39.9, 40.0);  // 北京附近
        double base_lon = rng.generateFloat(116.3, 116.4);
        double base_alt = rng.generateFloat(10.0, 30.0);
        
        // 每个无人机对该目标的观测
        for(int uav = 0; uav < num_uavs; uav++) {
            Package pkg(current_time);
            
            // 设置无人机ID和相机ID
            pkg.uav_id = "UAV_" + std::to_string(uav);
            pkg.camera_id = uav;
            
            // 随机生成目标类别（人或车）
            pkg.class_id = rng.generateInt(0, 1);
            pkg.class_name = pkg.class_id == 0 ? "person" : "vehicle";
            
            // 设置跟踪器ID
            pkg.tracker_id = target;
            
            // 添加观测噪声生成观测位置
            pkg.location = {
                static_cast<float>(base_lat + rng.generateFloat(-0.0001, 0.0001)),
                static_cast<float>(base_lon + rng.generateFloat(-0.0001, 0.0001)),
                static_cast<float>(base_alt + rng.generateFloat(-1.0, 1.0))
            };
            
            // 模拟相机参数
            pkg.camera_pose = {0.0f, -45.0f, 0.0f, 0.0f, 0.0f, 100.0f};  // 简化的相机姿态
            pkg.camera_K = {1000.0f, 1000.0f, 960.0f, 540.0f};  // 简化的相机内参
            pkg.camera_distortion = {0.0f, 0.0f, 0.0f, 0.0f};  // 假设无畸变
            
            // 模拟边界框
            pkg.Bbox = {rng.generateInt(0, 1920), rng.generateInt(0, 1080), 100, 200};
            

            
            packages.push_back(pkg);
        }
    }
    
    return packages;
}

// 打印Package信息的辅助函数
void printPackageInfo(const Package& pkg) {
    std::cout << "UAV: " << pkg.uav_id
              << ", Class: " << pkg.class_name
              << ", Tracker ID: " << pkg.tracker_id
              << ", Local ID: " << pkg.local_id
              << ", Global ID: " << pkg.global_id
              << "\nLocation: [" 
              << std::fixed << std::setprecision(6)
              << pkg.location[0] << ", "
              << pkg.location[1] << ", "
              << std::setprecision(2)
              << pkg.location[2] << "]\n"
              << std::endl;
}

int main() {
    // 创建随机数生成器
    RandomGenerator rng;
    
    // 设置模拟参数
    const int NUM_TARGETS = 3;  // 目标数量
    const int NUM_UAVS = 4;     // 无人机数量
    const double DISTANCE_THRESHOLD = 1;  // 约30米的经纬度差
    const int MAX_HISTORY = 10;  // 历史记录最大长度
    
    // 创建空间滤波器实例
    SpatialFilter filter(1000, DISTANCE_THRESHOLD, MAX_HISTORY, MAX_HISTORY);
    
    // 生成模拟数据
    std::vector<Package> packages = generateSimulatedData(rng, NUM_TARGETS, NUM_UAVS);
    
    std::cout << "原始数据：" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    for(const auto& pkg : packages) {
        printPackageInfo(pkg);
    }
    
    // 执行空间滤波
    std::vector<Package> filtered_packages = filter.process(packages);
    
    std::cout << "滤波后数据：" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    for(const auto& pkg : filtered_packages) {
        printPackageInfo(pkg);
    }
    
    // 模拟后续帧处理
    std::cout << "处理后续帧..." << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    // 生成新的观测数据并处理
    for(int frame = 1; frame < 3; frame++) {
        std::cout << "Frame " << frame << ":" << std::endl;
        std::vector<Package> new_packages = generateSimulatedData(rng, NUM_TARGETS, NUM_UAVS);
        std::vector<Package> new_filtered = filter.process(new_packages);
        
        for(const auto& pkg : new_filtered) {
            printPackageInfo(pkg);
        }
        std::cout << "----------------------------------------" << std::endl;
    }
    
    return 0;
}

