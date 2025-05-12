#include "modules/RTK_difference.h"
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <set>

RTKDifference::RTKDifference(double time_slice, int class_1, double x1, double y1, int class_2, double x2, double y2, int max_queue_length) : Difference("RTKDifference", time_slice, max_queue_length),
                                                                            max_queue_length(max_queue_length)
{
    configureClass(class_1, {x1, y1});
    configureClass(class_2, {x2, y2});
}


void RTKDifference::configureClass(int cls_id, std::pair<double, double> pos) {
    class_config[cls_id] = pos;
}

std::vector<Package> RTKDifference::process(const std::vector<Package> &packages)  // 噪声是前随机数
{
    // Step 1: 检查是否存在基准类别（class_config中的类别）数据包
    bool has_base_class = std::any_of(packages.begin(), packages.end(),
        [this](const Package& pkg) {
            return class_config.find(pkg.class_id) != class_config.end();
        });
    if (!has_base_class) return {}; // 无基准类别直接返回
    std::vector<Package> result;
    // 存储基准类别位置信息的结构体
    struct BaseClassPosRecord {
        std::vector<double> original_pos; // 原始坐标
        std::vector<double> adjusted_pos; // 调整后的坐标
    };
    std::vector<BaseClassPosRecord> base_class_records;
    // 初始化随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand_offset(-0.25, 0.25);
    // Step 2: 处理基准类别数据包
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
            processed_pkg.location[0] = base_x + rand_offset(gen);
            processed_pkg.location[1] = base_y + rand_offset(gen);
            
            // 保存位置记录供后续使用
            base_class_records.push_back({orig_pos, processed_pkg.location});
        }
        result.push_back(processed_pkg);
    }
    // Step 3: 处理非基准类别数据包
    for (Package& pkg : result) {
        if (class_config.count(pkg.class_id)) continue; // 跳过基准类别
        
        // 确保位置向量有效
        if (pkg.location.size() < 2) {
            pkg.location.resize(2, 0.0);
        }
        const std::vector<double>& current_pos = pkg.location;
        // 查找最近基准点的原始位置
        const BaseClassPosRecord* nearest_record = nullptr;
        double min_sq_distance = std::numeric_limits<double>::max();
        for (const auto& record : base_class_records) {
            if (record.original_pos.size() < 2) continue;
            // 计算欧氏距离平方（避免开方提升性能）
            double dx = current_pos[0] - record.original_pos[0];
            double dy = current_pos[1] - record.original_pos[1];
            double sq_dist = dx*dx + dy*dy;
            if (sq_dist < min_sq_distance) {
                min_sq_distance = sq_dist;
                nearest_record = &record;
            }
        }
        // 应用位置调整
        if (nearest_record && nearest_record->adjusted_pos.size() >= 2) {
            // 计算原始相对偏移
            double dx = current_pos[0] - nearest_record->original_pos[0];
            double dy = current_pos[1] - nearest_record->original_pos[1];
            
            // 新位置 = 基准点调整后的位置 + 原始偏移量
            pkg.location[0] = nearest_record->adjusted_pos[0] + dx;
            pkg.location[1] = nearest_record->adjusted_pos[1] + dy;
        }
    }
    return result;
}

// std::vector<Package> RTKDifference::process(const std::vector<Package> &packages)  // 噪声是前一帧和这一帧的差
// {
//     // Step 1: 检查是否存在基准类别（class_config中的类别）数据包
//     bool has_base_class = std::any_of(packages.begin(), packages.end(),
//         [this](const Package& pkg) {
//             return class_config.find(pkg.class_id) != class_config.end();
//         });
//     if (!has_base_class) return {}; // 无基准类别直接返回
//     std::vector<Package> result;
//     // 存储基准类别位置信息的结构体
//     struct BaseClassPosRecord {
//         std::vector<double> original_pos; // 原始坐标
//         std::vector<double> adjusted_pos; // 调整后的坐标
//     };
//     std::vector<BaseClassPosRecord> base_class_records;

//     // Step 2: 处理基准类别数据包
//     for (const Package& pkg : packages) {
//         Package processed_pkg = pkg.copy();
        
//         if (class_config.count(processed_pkg.class_id)) {
//             // 获取当前类别的基准坐标
//             auto [base_x, base_y] = class_config.at(processed_pkg.class_id);
            
//             // 记录原始位置
//             std::vector<double> orig_pos = processed_pkg.location;
            
//             // 计算噪声：上一次位置和当前位置的差值作为噪声
//             double noise_x = 0.0, noise_y = 0.0;
//             if (last_original_pos.count(processed_pkg.class_id)) {
//                 noise_x = orig_pos[0] - last_original_pos[processed_pkg.class_id].first;
//                 noise_y = orig_pos[1] - last_original_pos[processed_pkg.class_id].second;
//             }

//             // 更新last_original_pos
//             last_original_pos[processed_pkg.class_id] = {orig_pos[0], orig_pos[1]};

//             // 生成新位置：基准坐标 + 位置差异噪声
//             if (processed_pkg.location.size() < 2) {
//                 processed_pkg.location.resize(2, 0.0);
//             }

//             processed_pkg.location[0] = base_x + noise_x;
//             processed_pkg.location[1] = base_y + noise_y;
            
//             // 保存位置记录供后续使用
//             base_class_records.push_back({orig_pos, processed_pkg.location});
//         }
//         result.push_back(processed_pkg);
//     }
//     // Step 3: 处理非基准类别数据包
//     for (Package& pkg : result) {
//         if (class_config.count(pkg.class_id)) continue; // 跳过基准类别
        
//         // 确保位置向量有效
//         if (pkg.location.size() < 2) {
//             pkg.location.resize(2, 0.0);
//         }
//         const std::vector<double>& current_pos = pkg.location;
//         // 查找最近基准点的原始位置
//         const BaseClassPosRecord* nearest_record = nullptr;
//         double min_sq_distance = std::numeric_limits<double>::max();
//         for (const auto& record : base_class_records) {
//             if (record.original_pos.size() < 2) continue;
//             // 计算欧氏距离平方（避免开方提升性能）
//             double dx = current_pos[0] - record.original_pos[0];
//             double dy = current_pos[1] - record.original_pos[1];
//             double sq_dist = dx*dx + dy*dy;
//             if (sq_dist < min_sq_distance) {
//                 min_sq_distance = sq_dist;
//                 nearest_record = &record;
//             }
//         }
//         // 应用位置调整
//         if (nearest_record && nearest_record->adjusted_pos.size() >= 2) {
//             // 计算原始相对偏移
//             double dx = current_pos[0] - nearest_record->original_pos[0];
//             double dy = current_pos[1] - nearest_record->original_pos[1];
            
//             // 新位置 = 基准点调整后的位置 - 原始偏移量
//             pkg.location[0] = nearest_record->adjusted_pos[0] + dx;
//             pkg.location[1] = nearest_record->adjusted_pos[1] + dy;
//         }
//     }
//     return result;
// }