#include "modules/time_filter.h"
#include <algorithm>


TimeFilter::TimeFilter(double time_slice, int max_queue_length): 
                                    PreProcess::PreProcess("TimeFilter", time_slice, max_queue_length),
                                    max_queue_length(max_queue_length){}

std::vector<Package> TimeFilter::process(std::vector<Package>& data) {
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
                new_pkg.dp = exist_pkg.dp;
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