#pragma once
#include <string>
#include <vector>
#include <list>
#include <memory>
#include <mutex>
#include <ctime>
#include <variant>
#include <atomic>

// Package类用于存储检测和跟踪信息
class Package {
public:
    Package(time_t time = 0);
    
    // 只读成员
    time_t time;
    std::string uav_id = "";
    int camera_id = -1;
    std::vector<float> camera_pose;  // [yaw,pitch,roll,x,y,z]
    std::vector<float> camera_K;     // [fx,fy,cx,cy]
    std::vector<float> camera_distortion;  // [k1,k2,p1,p2]
    std::vector<int> Bbox;           // [x,y,w,h]
    std::vector<float> norm_Bbox;    // [x,y,w,h] 归一化后的bbox
    int class_id = -1;                    // 0:人 1:车
    std::string class_name = "";
    int tracker_id = -1;
    std::vector<float> uav_wgs;      // WGS84 [lat,lon,alt]
    std::vector<float> uav_utm;      // UTM坐标
    std::string obj_img = "";
    
    // 用于评估
    int uid = -1;
    
    // 用于时间切片
    int delim_flag = 0;
    
    // 读写成员
    int global_id = -1;
    int local_id = -1;
    std::vector<float> location;     // WGS84
    
    // 方法
    std::vector<float> getCenterPoint() const;
    Package copy() const;
    void setBboxType(const std::string& type);
    std::string toString() const;

private:
    std::string bbox_type;  // "cxcywh", "xyxy", 或 "xywh"
};

// 基于时间优先级的队列
class TimePriorityQueue {
public:
    TimePriorityQueue(size_t maxCount = 0);

    bool isEmpty() const;
    bool isFull() const;
    int push(const Package& package);
    Package pop();
    void clear();
    std::vector<Package> getTimeSlice(double timeSlice);
    double deltaTime() const;
    void setMaxCount(size_t maxCount);
    size_t size() const;
    std::string toString() const;
    
    // 运算符重载
    Package& operator[](size_t index);
    const Package& operator[](size_t index) const;

    // 迭代器支持
    using iterator = std::list<Package>::iterator;
    using const_iterator = std::list<Package>::const_iterator;
    iterator begin() { return queue.begin(); }
    iterator end() { return queue.end(); }
    const_iterator begin() const { return queue.begin(); }
    const_iterator end() const { return queue.end(); }

private:
    std::list<Package> queue;
    size_t maxCount;
};

// Module基类
class Module {
public:
    Module(const std::string& name, size_t maxQueueLength = 0);
    virtual ~Module() = default;

    void setInputLock(std::shared_ptr<std::mutex> lock);
    void setOutputLock(std::shared_ptr<std::mutex> lock);
    void setInputQueue(std::shared_ptr<TimePriorityQueue> inputQueue);
    void setOutputQueue(std::shared_ptr<TimePriorityQueue> outputQueue);
    
    virtual void run() = 0;
    virtual void close();
    std::string toString() const;

protected:
    std::string name;
    std::shared_ptr<TimePriorityQueue> inputQueue;
    std::shared_ptr<TimePriorityQueue> outputQueue;
    std::shared_ptr<std::mutex> inputLock;
    std::shared_ptr<std::mutex> outputLock;
    size_t maxQueueLength;
    std::atomic<bool> isRunning;
};

