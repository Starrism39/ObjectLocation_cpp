#pragma once
#include <string>
#include <vector>
#include <list>
#include <memory>
#include <mutex>
#include <ctime>
#include <variant>
#include <atomic>
#include <iostream>

#include "input/package.h"

// Package类用于存储检测和跟踪信息
class Package {
public:
    Package(time_t time = 0);
    
    // 只读成员，由相机DataPackage类得到的数据
    time_t time;
    std::string uav_id = "";
    int camera_id = -1;
    std::vector<double> camera_pose;  // [yaw,pitch,roll,x,y,z]
    std::vector<double> camera_K;     // [fx,fy,cx,cy]
    std::vector<double> camera_distortion;  // [k1,k2,p1,p2,k3]
    std::vector<int> Bbox;           // [x,y,w,h]
    std::vector<double> norm_Bbox;    // [x,y,w,h] 归一化后的bbox
    float prob = 0.0;                        // 置信度
    int class_id = -1;
    std::string class_name = "";
    int tracker_id = -1;
    std::vector<double> uav_utm;      // UTM坐标
    std::shared_ptr<DataPackage> dp;
    

    // 以下成员全是本模块后续需要填入的参数
    // 用于评估
    int uid = -1;
    
    // 用于时间切片
    int delim_flag = 0;
    
    // 读写成员
    int global_id = -1;
    int local_id = -1;
    std::vector<double> location;     // 目标UTM
    
    // 方法
    std::vector<double> getCenterPoint() const;
    Package copy() const;
    void setBboxType(const std::string& type);
    std::string toString() const;

private:
    std::string bbox_type;  // "cxcywh", "xyxy", 或 "xywh"
};


// OutPackage用于处理后包的合并
struct Object{
    int global_id;
    std::vector<double> location;
    std::vector<std::map<int, cv::Mat>> uav_img;
};

struct OutPackage{
    time_t time;
    std::vector<Object> objs;
};


// 基于时间优先级的队列
template<typename T>
class TimePriorityQueue {
public:
    TimePriorityQueue(size_t maxCount = 0) 
        : queue(std::list<T>()), maxCount(maxCount) {}

    bool isEmpty() const {
        return queue.empty();
    }

    bool isFull() const {
        return maxCount > 0 && queue.size() >= maxCount;
    }

    int push(const T& element) {
        if (isFull()) return -1;
        
        // 按时间降序插入（假设T有time成员）
        auto it = queue.begin();
        while (it != queue.end() && it->time >= element.time) {
            ++it;
        }
        queue.insert(it, element);
        return 0;
    }
     
    T pop() {
        if (isEmpty()) throw std::runtime_error("TimePriorityQueue is empty");
        T element = queue.back();
        queue.pop_back();
        return element;
    }
     
    T peek() {
        if (isEmpty()) throw std::runtime_error("TimePriorityQueue is empty");
        return queue.back();
    }
     
    void clear() {
        queue.clear();
    }
     
    // 下采样
    std::vector<T> getTimeSlice(double timeSlice) {
        if (isEmpty()) {
            throw std::runtime_error("TimePriorityQueue is empty");
        }

        double stopTime = queue.back().time + timeSlice;  // 默认值为最小时间+时间间隔

        // 找到分界点
        auto splitIt = std::find_if(queue.begin(), queue.end(),
            [stopTime](const T& element) { return element.time <= stopTime; });

        // 提取时间片段并返回
        std::vector<T> timeSliceList(splitIt, queue.end());
        queue.erase(splitIt, queue.end());
        return timeSliceList;
    }

    // 步长为1的滑动窗口
    std::vector<T> getSlidingWindow(double timeSlice) {
        if (isEmpty()) {
            throw std::runtime_error("TimePriorityQueue is empty");
        }
    
        double stopTime = queue.back().time + timeSlice;  // 默认值为最小时间+时间间隔

        // 找到分界点
        auto splitIt = std::find_if(queue.begin(), queue.end(),
            [stopTime](const T& element) { return element.time <= stopTime; });
    
        // 生成时间窗口内的数据副本
        std::vector<T> timeSliceList(splitIt, queue.end());
    
        // 仅弹出队列最后一个元素
        if (!queue.empty()) {
            queue.pop_back();
        }
    
        return timeSliceList;
    }
    
     
    double deltaTime() const {
        if (queue.size() < 2) {
            return 0.0;
        }
        return queue.front().time - queue.back().time;
    }
     
    void setMaxCount(size_t maxCount) {
        this->maxCount = maxCount;
    }
     
    size_t size() const {
        return queue.size();
    }
     
    T& operator[](size_t index) {
        auto it = queue.begin();
        std::advance(it, index);
        return *it;
    }
     
    const T& operator[](size_t index) const {
        auto it = queue.begin();
        std::advance(it, index);
        return *it;
    }
     
    std::string  toString() const {
        std::ostringstream oss;
        for (const auto& element : queue) {
            oss << element.toString() << " ";  // 要求T类型有toString()方法
        }
        return oss.str();
    }

    // 迭代器支持
    using iterator = typename std::list<T>::iterator;          // 注意 typename
    using const_iterator = typename std::list<T>::const_iterator;
    iterator begin() { return queue.begin(); }
    iterator end() { return queue.end(); }
    const_iterator begin() const { return queue.begin(); }
    const_iterator end() const { return queue.end(); }

private:
    std::list<T> queue;      // 底层容器类型改为 list<T>
    size_t maxCount;
};

// Module基类
class Module {
public:
    Module(const std::string& name, size_t maxQueueLength = 0);
    virtual ~Module() = default;

    void setInputLock(std::shared_ptr<std::mutex> lock);
    void setOutputLock(std::shared_ptr<std::mutex> lock);
    void setInputQueue(std::shared_ptr<TimePriorityQueue<Package>> inputQueue);
    void setOutputQueue(std::shared_ptr<TimePriorityQueue<Package>> outputQueue);
    std::shared_ptr<TimePriorityQueue<Package>> getOutputQueue();
    std::shared_ptr<std::mutex> getOutputLock();
    
    virtual void run() = 0;
    virtual void close();
    std::string toString() const;

protected:
    std::string name;
    std::shared_ptr<TimePriorityQueue<Package>> inputQueue;
    std::shared_ptr<TimePriorityQueue<Package>> outputQueue;
    std::shared_ptr<std::mutex> inputLock;
    std::shared_ptr<std::mutex> outputLock;
    size_t maxQueueLength;
    std::atomic<bool> isRunning;
};


// 辅助函数：打印Package内容
template <typename T>
void PrintVector(const std::string &name, const std::vector<T> &vec);
void PrintPackage(const Package &pkg);
std::string type2str(int type);
void printOutPackage(const OutPackage& package);