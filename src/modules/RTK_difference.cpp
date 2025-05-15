#include "modules/RTK_difference.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <set>

RTKDifference::RTKDifference(double time_slice, int class_1, double x1, double y1, int max_queue_length) : Difference("RTKDifference", time_slice, max_queue_length),
                                                                            class_1_(class_1),
                                                                            max_queue_length(max_queue_length)
{
    configureClass(class_1, {x1, y1});
    del_class_config_1[class_1] = {0, 0};
    del_class_config_2[class_1] = {0, 0};
}


void RTKDifference::configureClass(int cls_id, std::pair<double, double> pos) {
    class_config[cls_id] = pos;
}

std::vector<Package> RTKDifference::process(const std::vector<Package> &packages)
{
    std::vector<Package> result;

    // Step 1: 处理基准类别数据包
    for (const Package& pkg : packages) {
        Package processed_pkg = pkg.copy();
        
        if (class_config.count(processed_pkg.class_id)) {
            // 获取当前类别的基准坐标
            auto [base_x, base_y] = class_config.at(processed_pkg.class_id);
            
            // 记录原始位置
            std::vector<double> orig_pos = processed_pkg.location;
            
            // 生成新位置：基准坐标 + 随机扰动
            if (processed_pkg.location.size() < 2) {
                processed_pkg.location.resize(2, 0.0);
            }

            // 记录位置差
            std::vector<double> del_pos = {orig_pos[0] - base_x, orig_pos[1] - base_y};
            
            // 根据UAV ID选择保存位置差的map
            if (processed_pkg.uav_id == "0") {
                del_class_config_1[processed_pkg.class_id] = {del_pos[0], del_pos[1]};
            } else if (processed_pkg.uav_id == "1") {
                del_class_config_2[processed_pkg.class_id] = {del_pos[0], del_pos[1]};
            }

            // 更改processed_pkg的location
            std::random_device rd;
            std::mt19937 gen_(rd());
            std::uniform_real_distribution<double> dis_(-0.2, 0.2);
            processed_pkg.location[0] = base_x + dis_(gen_);
            processed_pkg.location[1] = base_y + dis_(gen_);
        }
        result.push_back(processed_pkg);
    }
    
    // Step 2: 处理非基准类别数据包
    for (Package& pkg : result) {
        if (class_config.count(pkg.class_id)) continue; // 跳过基准类别

        std::pair<double, double> del_pos;

        // 根据UAV ID选择使用的位置差
        if (pkg.uav_id == "0") {
            auto del_config = del_class_config_1.at(class_1_);
            del_pos = del_config;

        }
        else if (pkg.uav_id == "1") {
            auto del_config = del_class_config_2.at(class_1_);
            del_pos = del_config;

        }
        
        // 确保位置向量有效
        if (pkg.location.size() < 2) {
            pkg.location.resize(2, 0.0);
        }
        
        pkg.location[0] = pkg.location[0] - del_pos.first;
        pkg.location[1] = pkg.location[1] - del_pos.second;

    }
    return result;
}

// std::vector<Package> RTKDifference::process(const std::vector<Package> &packages)
// {
//     std::vector<Package> result;

//     // Step 1: 处理基准类别数据包
//     for (const Package& pkg : packages) {
//         Package processed_pkg = pkg.copy();
        
//         if (class_config.count(processed_pkg.class_id)) {
//             // 获取当前类别的基准坐标
//             auto [base_x, base_y] = class_config.at(processed_pkg.class_id);
            
//             // 记录原始位置
//             std::vector<double> orig_pos = processed_pkg.location;
            
//             // 生成新位置：基准坐标 + 随机扰动
//             if (processed_pkg.location.size() < 2) {
//                 processed_pkg.location.resize(2, 0.0);
//             }

//             // 记录位置差
//             std::vector<double> del_pos = {orig_pos[0] - base_x, orig_pos[1] - base_y};
            
//             // 保存位置差
//             del_class_config_1[processed_pkg.class_id] = {del_pos[0], del_pos[1]};

//             // 更改processed_pkg的location
//             std::random_device rd;
//             std::mt19937 gen_(rd());
//             std::uniform_real_distribution<double> dis_(-0.2, 0.2);
//             processed_pkg.location[0] = base_x + dis_(gen_);
//             processed_pkg.location[1] = base_y + dis_(gen_);
//         }
//         result.push_back(processed_pkg);
//     }

//     // Step 2: 处理非基准类别数据包
//     for (Package& pkg : result) {
//         if (class_config.count(pkg.class_id)) continue; // 跳过基准类别

//         auto [del_x, del_y] = del_class_config_1.at(class_1_);
        
//         // 确保位置向量有效
//         if (pkg.location.size() < 2) {
//             pkg.location.resize(2, 0.0);
//         }
        
//         pkg.location[0] = pkg.location[0] - del_x;
//         pkg.location[1] = pkg.location[1] - del_y;

//     }
//     return result;
// }