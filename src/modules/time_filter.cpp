#include "modules/time_filter.h"
#include <algorithm>

TimeFilter::TimeFilter(int max_queue_length): max_queue_length(max_queue_length){}

std::vector<Package> TimeFilter::process(const std::vector<Package>& data) {
    std::unordered_map<std::string, Package> temp_map;

    // 逆序处理数据包
    for (auto it = data.rbegin(); it != data.rend(); ++it) {
        const Package& pkg = *it;
        std::string map_key = pkg.uav_id + "_" + std::to_string(pkg.tracker_id);

        auto existing = temp_map.find(map_key);
        if (existing == temp_map.end()) {
            temp_map.emplace(map_key, pkg.copy());
        } else {
            Package& exist_pkg = existing->second;
            if (exist_pkg.time < pkg.time) {
                Package new_pkg = pkg.copy();
                // 保留旧数据的图像
                if (!exist_pkg.obj_img.empty()) {
                    new_pkg.obj_img = exist_pkg.obj_img;
                }
                temp_map[map_key] = new_pkg;
            }
        }
    }

    // 构建结果集
    std::vector<Package> result;
    result.reserve(temp_map.size());
    for (const auto& pair : temp_map) {
        result.emplace_back(pair.second);
    }

    return result;
}