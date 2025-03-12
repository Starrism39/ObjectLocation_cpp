#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>

#include"framework/filter.h"
class CircularQueue {
private:
    int capacity;
    std::vector<std::map<std::string, int>> queue;  // 改用vector存储map
    int size;
    int front;
    int rear;
    int max_global_id;

public:
    CircularQueue(int capacity);
    
    bool isEmpty() const;
    bool isFull() const;
    
    void enqueue(const std::map<std::string, int>& item);
    std::map<std::string, int> dequeue();
    
    void display() const;
    
    // 获取最新数据
    std::map<std::string, int> getLastElement() const;
    // 获取第i新的数据
    std::map<std::string, int> getLastElementI(int i) const;
    
    // 获取和设置max_global_id
    int getMaxGlobalId() const;
    void setMaxGlobalId(int id);

    // 查找key是否存在于指定索引的map中
    bool findKeyInMap(const std::string& key, int index, int& value) const;

    // 获取size
    int getSize() const { return size; }

    
    std::vector<std::map<std::string, int>>& getQueue() { return queue; }
};

class SpatialFilter : public Filter{

public:
    SpatialFilter(double time_slice, double distance_threshold, int max_map, int parallel = 1, int max_queue_length = 0);
    std::vector<Package> process(const std::vector<Package>& packages) override;

private:
    double distance_threshold;
    int max_map;
    int max_queue_length;
    CircularQueue global_history;

    std::vector<std::vector<std::vector<Package>>> classifyClassIdUav(const std::vector<Package>& packages);
    std::pair<std::vector<std::vector<Package>>, int> spatialFilter1(double distance_threshold, std::vector<std::vector<Package>>& detections_list, int local_id);
    std::map<int, std::vector<Package>> spatialFilter2(const std::vector<std::vector<std::vector<Package>>>& class_list);
    std::vector<Package> findGlobal(std::map<int, std::vector<Package>>& grouped_detections);

};
