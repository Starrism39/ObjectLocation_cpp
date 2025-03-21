#include "modules/time_filter.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <ctime>

// 用于生成随机数的辅助函数
class RandomGenerator
{
private:
    std::random_device rd;
    std::mt19937 gen;

public:
    RandomGenerator() : gen(rd()) {}

    double generateFloat(double min, double max)
    {
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }

    int generateInt(int min, int max)
    {
        std::uniform_int_distribution<> dis(min, max);
        return dis(gen);
    }
};

// 生成模拟的Package数据
std::vector<Package> generateSimulatedData(RandomGenerator &rng, int num_targets, int num_uavs, time_t base_time)
{
    std::vector<Package> packages;

    for (int target = 0; target < num_targets; target++)
    {
        // 随机生成目标的真实位置
        double base_lat = rng.generateFloat(39.9, 40.0);
        double base_lon = rng.generateFloat(116.3, 116.4);
        double base_alt = rng.generateFloat(10.0, 30.0);

        for (int uav = 0; uav < num_uavs; uav++)
        {
            // 为每个无人机生成多个时间点的观测
            for (int t = 0; t < 3; t++)
            {
                Package pkg(base_time + t);

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
                    static_cast<float>(base_alt + rng.generateFloat(-1.0, 1.0))};

                // 模拟相机参数
                pkg.camera_pose = {0.0f, -45.0f, 0.0f, 0.0f, 0.0f, 100.0f}; // 简化的相机姿态
                pkg.camera_K = {1000.0f, 1000.0f, 960.0f, 540.0f};          // 简化的相机内参
                pkg.camera_distortion = {0.0f, 0.0f, 0.0f, 0.0f};           // 假设无畸变

                // 模拟边界框
                pkg.Bbox = {rng.generateInt(0, 1920), rng.generateInt(0, 1080), 100, 200};

                packages.push_back(pkg);
            }
        }
    }

    return packages;
}

// 打印Package信息的辅助函数
void printPackageInfo(const Package &pkg)
{
    std::cout << "Time: " << pkg.time
              << ", UAV: " << pkg.uav_id
              << ", Tracker ID: " << pkg.tracker_id
              << ", Class: " << pkg.class_name
              << "\nLocation: ["
              << std::fixed << std::setprecision(6)
              << pkg.location[0] << ", "
              << pkg.location[1] << ", "
              << std::setprecision(2)
              << pkg.location[2] << "]\n"
              << std::endl;
}

int main()
{
    RandomGenerator rng;

    // 设置参数
    const int NUM_TARGETS = 2; // 目标数量
    const int NUM_UAVS = 3;    // 无人机数量
    const int MAX_QUEUE_LENGTH = 10;
    time_t base_time = std::time(nullptr);

    // 创建时间滤波器实例
    TimeFilter filter(1.0, MAX_QUEUE_LENGTH);

    // 生成并处理第一帧数据
    std::cout << "处理第一帧数据：" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    std::vector<Package> packages = generateSimulatedData(rng, NUM_TARGETS, NUM_UAVS, base_time);

    std::cout << "原始数据：" << std::endl;
    for (const auto &pkg : packages)
    {
        printPackageInfo(pkg);
    }

    std::vector<Package> filtered_packages = filter.process(packages);

    std::cout << "\n滤波后数据：" << std::endl;
    for (const auto &pkg : filtered_packages)
    {
        printPackageInfo(pkg);
    }

    // 模拟处理后续帧
    std::cout << "\n处理后续帧..." << std::endl;
    for (int frame = 1; frame < 3; frame++)
    {
        std::cout << "\nFrame " << frame << ":" << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        // 生成新的时间点的数据
        std::vector<Package> new_packages = generateSimulatedData(rng, NUM_TARGETS, NUM_UAVS, base_time + frame * 3);
        std::vector<Package> new_filtered = filter.process(new_packages);

        for (const auto &pkg : new_filtered)
        {
            printPackageInfo(pkg);
        }
    }

    return 0;
}
