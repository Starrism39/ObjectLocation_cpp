#include "modules/spatial_filter.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <set>

CircularQueue::CircularQueue(int cap) : 
    capacity(cap),
    queue(capacity),
    size(0),
    front(0),
    rear(-1),
    max_global_id(-1) {
}


bool CircularQueue::isEmpty() const {
    return size == 0;
}

bool CircularQueue::isFull() const {
    return size == capacity;
}

void CircularQueue::enqueue(const std::map<std::string, int>& item) {
    if (isFull()) {
        dequeue();
    }
    rear = (rear + 1) % capacity;
    queue[rear] = item;
    size++;
}

std::map<std::string, int> CircularQueue::dequeue() {
    if (isEmpty()) {
        return std::map<std::string, int>();
    }
    
    std::map<std::string, int> item = queue[front];
    queue[front].clear();
    front = (front + 1) % capacity;
    size--;
    return item;
}

void CircularQueue::display() const {
    std::cout << "Current Queue: \n";
    int index = front;
    for (int i = 0; i < size; i++) {
        std::cout << "Index " << index << ":\n";
        for (const auto& [key, value] : queue[index]) {
            std::cout << key << ": " << value << "\n";
        }
        index = (index + 1) % capacity;
    }
    std::cout << std::endl;
}

std::map<std::string, int> CircularQueue::getLastElement() const {
    if (isEmpty()) {
        return std::map<std::string, int>();
    }
    return queue[rear];
}

std::map<std::string, int> CircularQueue::getLastElementI(int i) const {
    if (isEmpty() || i >= size || i < 0) {
        return std::map<std::string, int>();
    }
    int index = (rear - i + capacity) % capacity;
    return queue[index];
}

bool CircularQueue::findKeyInMap(const std::string& key, int index, int& value) const {
    if (index >= size || index < 0) {
        return false;
    }
    
    int actual_index = (rear - index + capacity) % capacity;
    auto it = queue[actual_index].find(key);
    if (it != queue[actual_index].end()) {
        value = it->second;
        return true;
    }
    return false;
}

int CircularQueue::getMaxGlobalId() const {
    return max_global_id;
}

void CircularQueue::setMaxGlobalId(int id) {
    max_global_id = id;
}

SpatialFilter::SpatialFilter(double distance_threshold ,int max_map, int max_queue_length):
    distance_threshold(distance_threshold),
    max_map(max_map),
    max_queue_length(max_queue_length),
    global_history(max_map){
}

std::vector<std::vector<std::vector<Package>>> SpatialFilter::classifyClassIdUav(const std::vector<Package>& packages){
    // 将package包队列按class_id拆分
    std::map<int, std::vector<Package>> group_cls_id;
    for(const auto& package : packages){
        group_cls_id[package.class_id].push_back(package);
    }

    std::vector<std::vector<std::vector<Package>>> class_list;

    for(const auto& [_, pkgs] : group_cls_id){
        // 将package包队列按uav_id拆分
        std::map<std::string, std::vector<Package>> group_uav_id;
        for(const auto& pkg : pkgs){
            group_uav_id[pkg.uav_id].push_back(pkg);
        }

        // 底端按uav_id分类
        std::vector<std::vector<Package>>  uav_groups;
        for(const auto& [_, uav_packages] : group_uav_id){
            uav_groups.push_back(uav_packages);
        }

        // 顶端按class_id分类
        class_list.push_back(uav_groups);
    }
    return class_list;
}

std::pair<std::vector<std::vector<Package>>, int> SpatialFilter::spatialFilter1(double distance_threshold, std::vector<std::vector<Package>>& detections_list, int local_id){
    // 对于相同类别下两两为一组的无人机，进行空间过滤
    for(size_t i = 0; i < detections_list.size(); i++){
        auto& list_i = detections_list[i];
        for(size_t j = i + 1; j < detections_list.size(); j++){
            auto &list_j = detections_list[j];
            Eigen::MatrixXd matrix_distance = Eigen::MatrixXd::Constant(list_i.size(), list_j.size(), std::numeric_limits<double>::infinity());

            // 对于这组无人机下的目标计算距离，小于阈值的赋值并更新下标
            for(size_t index_i = 0; index_i < list_i.size(); index_i++){
                if(list_i[index_i].local_id == -1){
                    list_i[index_i].local_id = local_id++;
                }
                for(size_t index_j = 0; index_j < list_j.size(); index_j++){
                    if (!(list_i[index_i].local_id != -1 && 
                        list_j[index_j].local_id != -1)) {

                        Eigen::Vector3d loc_i;
                        loc_i << list_i[index_i].location[0], 
                                    list_i[index_i].location[1], 
                                    list_i[index_i].location[2];

                        Eigen::Vector3d loc_j;
                        loc_j << list_j[index_j].location[0], 
                                    list_j[index_j].location[1], 
                                    list_j[index_j].location[2];

                        double distance = (loc_i - loc_j).norm();
                        
                        if (distance < distance_threshold) {
                            matrix_distance(index_i, index_j) = distance;
                        }
                    }
                }
            }
            
            // 找到最小距离并更新local_id
            for(int k = 0; k < std::min(list_i.size(), list_j.size()); k++){
                Eigen::MatrixXd::Index minRow, minCol;
                double minVal = matrix_distance.minCoeff(&minRow, &minCol);
                if(minVal > 1000.0) break;
                list_j[minCol].local_id = list_i[minRow].local_id;
                // 将已匹配的行和列设置为无穷大
                matrix_distance.row(minRow).setConstant(
                    std::numeric_limits<double>::infinity());
                matrix_distance.col(minCol).setConstant(
                    std::numeric_limits<double>::infinity());
            }
        }
    }

    // 处理最后一个uav列表
    if (!detections_list.empty() && !detections_list.back().empty()) {
        for (auto& detect : detections_list.back()) {
            if (detect.local_id == -1) {
                detect.local_id = local_id++;
            }
        }
    }
    
    return {detections_list, local_id};
}
std::map<int, std::vector<Package>> SpatialFilter::spatialFilter2(const std::vector<std::vector<std::vector<Package>>>& class_list){
    std::map<int, std::vector<Package>> grouped_detections;

    // 展开数据结构为一维
    std::vector<Package> detections_list;
    for (const auto& class_group : class_list) {
        for (const auto& uav_group : class_group) {
            detections_list.insert(detections_list.end(), 
                                 uav_group.begin(), 
                                 uav_group.end());
        }
    }

    // 按local_id分组
    for (const auto& detect : detections_list) {
        grouped_detections[detect.local_id].push_back(detect);
    }

    // 计算平均位置
    for (auto& [local_id, group] : grouped_detections) {
        Eigen::Vector3d sum_location = Eigen::Vector3d::Zero();
        
        for (const auto& detect : group) {
            Eigen::Vector3d loc;
            loc << detect.location[0], 
                detect.location[1], 
                detect.location[2];

            sum_location += loc;
        }
        
        Eigen::Vector3d avg_location = sum_location / static_cast<double>(group.size());
        
        // 更新位置
        for (auto& detect : group) {
            detect.location = {
                (avg_location[0]), 
                (avg_location[1]), 
                (avg_location[2])
            };
        }
        
        // 按uav_id排序
        std::sort(group.begin(), group.end(), 
                    [](const Package& a, const Package& b) {
                        return a.uav_id < b.uav_id;
                    });
    }
    
    return grouped_detections;
}

std::vector<Package> SpatialFilter::findGlobal(std::map<int, std::vector<Package>>& grouped_detections) {
    std::map<int, std::map<int, std::vector<Package>>> class_local_groups;
    std::vector<Package> return_data;
    std::map<std::string, int> current_track;
    
    // 用于跟踪当前帧中每个class_id下已分配的global_id
    std::map<int, std::set<int>> class_global_ids;
    
    // 按class_id和local_id分组
    for(const auto& [local_id, group] : grouped_detections) {
        if(!group.empty()) {
            int class_id = group[0].class_id;
            class_local_groups[class_id][local_id] = group;
        }
    }

    // 对每个类别分别处理
    for(auto& [class_id, local_groups] : class_local_groups) {
        // 将local_groups转换为vector并按目标数量排序
        std::vector<std::pair<int, std::vector<Package>>> sorted_groups;
        for(const auto& [local_id, group] : local_groups) {
            sorted_groups.push_back({local_id, group});
        }
        
        // 按目标数量降序排序
        std::sort(sorted_groups.begin(), sorted_groups.end(),
            [](const auto& a, const auto& b) {
                return a.second.size() > b.second.size();
            });

        // 处理排序后的组
        for(auto& [local_id, group] : sorted_groups) {
            bool found = false;
            int found_global_id = -1;

            // 在历史记录中查找匹配
            for(int i = 0; i < global_history.getSize() && !found; i++) {
                for(auto& detect : group) {
                    std::string key = std::to_string(class_id) + "_" + 
                                    detect.uav_id + "_" + 
                                    std::to_string(detect.tracker_id);
                    int track_value;
                    if(global_history.findKeyInMap(key, i, track_value)) {
                        found = true;
                        found_global_id = track_value;
                        break;
                    }
                }
            }

            // 检查找到的global_id是否已在当前帧的其他local_id组中使用
            if(found && class_global_ids[class_id].count(found_global_id) > 0) {
                // 如果该global_id已被使用，分配新的global_id
                found = false;
            }

            // 如果没找到匹配或需要新的global_id
            if(!found) {
                found_global_id = global_history.getMaxGlobalId() + 1;
                global_history.setMaxGlobalId(found_global_id);
            }

            // 记录这个class_id下使用的global_id
            class_global_ids[class_id].insert(found_global_id);

            // 更新当前组的所有检测
            for(auto& detect : group) {
                detect.global_id = found_global_id;
                std::string key = std::to_string(class_id) + "_" + 
                                detect.uav_id + "_" + 
                                std::to_string(detect.tracker_id);
                current_track[key] = found_global_id;
            }

            return_data.insert(return_data.end(), group.begin(), group.end());
        }
    }

    // 更新历史记录
    global_history.enqueue(current_track);
    
    return return_data;
}

std::vector<Package> SpatialFilter::process(const std::vector<Package>& packages) {
    // 按class_id和uav_id分类
    auto class_list = classifyClassIdUav(packages);
    
    // 空间滤波1：分配local_id
    int local_id = 0;
    for (auto& class_group : class_list) {
        if (!class_group.empty()) {
            auto [updated_group, new_local_id] = 
                spatialFilter1(distance_threshold, class_group, local_id);
            class_group = updated_group;
            local_id = new_local_id;
        }
    }
    
    // 空间滤波2：更新平均位置
    auto grouped_list = spatialFilter2(class_list);
    
    // 执行global_id追踪
    return findGlobal(grouped_list);
}