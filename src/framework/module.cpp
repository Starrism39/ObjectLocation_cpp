#include "framework/module.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <sstream>


// Package实现
Package::Package(time_t time) : 
    time(time),
    camera_id(-1),
    class_id(-1),
    tracker_id(-1),
    uid(-1),
    delim_flag(0),
    global_id(-1),
    local_id(-1),
    bbox_type("xywh") {}


std::vector<double> Package::getCenterPoint() const {
    if (Bbox.size() < 4) {
        throw std::runtime_error("Bbox未正确初始化");
    }

    std::vector<double> center(2);
    if (bbox_type == "cxcywh") {
        center[0] = static_cast<double>(Bbox[0]);
        center[1] = static_cast<double>(Bbox[1] + Bbox[3]/2.0);
    }
    else if (bbox_type == "xyxy") {
        center[0] = static_cast<double>((Bbox[0] + Bbox[2])/2.0);
        center[1] = static_cast<double>((Bbox[1] + Bbox[3])/2.0);
    }
    else if (bbox_type == "xywh") {
        center[0] = static_cast<double>(Bbox[0] + Bbox[2]/2.0);
        center[1] = static_cast<double>(Bbox[1] + Bbox[3]/2.0);
    }
    else {
        throw std::runtime_error("bbox_type必须是 cxcywh、xyxy 或 xywh");
    }
    return center;
}

Package Package::copy() const {
    return Package(*this);  // 返回对象的副本
}

void Package::setBboxType(const std::string& type) {
    if (type != "cxcywh" && type != "xyxy" && type != "xywh") {
        throw std::runtime_error("bbox_type必须是 cxcywh、xyxy 或 xywh");
    }
    bbox_type = type;
}

std::string Package::toString() const {
    return "time:" + std::to_string(time);
}

TimePriorityQueue::TimePriorityQueue(size_t maxCount) 
    : queue(std::list<Package>()), maxCount(maxCount) {}

bool TimePriorityQueue::isEmpty() const {
    return queue.empty();
}

bool TimePriorityQueue::isFull() const {
    return maxCount > 0 && queue.size() >= maxCount;
}

int TimePriorityQueue::push(const Package& package) {
    if (isFull()) {
        return -1;
    }
    
    // 按时间降序插入（新的时间在前）
    auto it = queue.begin();
    while (it != queue.end() && it->time >= package.time) {
        ++it;
    }
    queue.insert(it, package);
    return 0;
}

Package TimePriorityQueue::pop() {
    if (isEmpty()) {
        throw std::runtime_error("TimePriorityQueue is empty");
    }
    Package package = queue.back();
    queue.pop_back();
    return package;
}

void TimePriorityQueue::clear() {
    queue.clear();
}

std::vector<Package> TimePriorityQueue::getTimeSlice(double timeSlice) {
    if (isEmpty()) {
        throw std::runtime_error("TimePriorityQueue is empty");
    }

    double stopTime = queue.back().time + timeSlice;  // 默认值为最小时间+时间间隔
    
    // 查找分界标志
    // 注：当有多个定位线程时不适用，因为队列顺序会被打乱！若无此逻辑可能会丢失最后两张图的Package
    // for (auto it = queue.rbegin(); it != queue.rend(); ++it) {
    //     if (it->delim_flag) {
    //         stopTime = it->time;
    //         break;
    //     }
    // }

    // 找到分界点
    auto splitIt = std::find_if(queue.begin(), queue.end(),
        [stopTime](const Package& p) { return p.time <= stopTime; });

    // if (splitIt != queue.end()) {
    //     splitIt->delim_flag = 1;
    // }

    // 提取时间片段并返回
    std::vector<Package> timeSliceList(splitIt, queue.end());
    queue.erase(splitIt, queue.end());
    return timeSliceList;
}

double TimePriorityQueue::deltaTime() const {
    if (queue.size() < 2) {
        return 0.0;
    }
    return queue.front().time - queue.back().time;
}

void TimePriorityQueue::setMaxCount(size_t maxCount) {
    this->maxCount = maxCount;
}

size_t TimePriorityQueue::size() const {
    return queue.size();
}

Package& TimePriorityQueue::operator[](size_t index) {
    auto it = queue.begin();
    std::advance(it, index);
    return *it;
}

const Package& TimePriorityQueue::operator[](size_t index) const {
    auto it = queue.begin();
    std::advance(it, index);
    return *it;
}

std::string TimePriorityQueue::toString() const {
    std::ostringstream oss;
    for (const auto& pkg : queue) {
        oss << pkg.toString() << " ";
    }
    return oss.str();
}

Module::Module(const std::string& name, size_t maxQueueLength)
    :name(name),
    inputQueue(nullptr),
    outputQueue(nullptr),
    inputLock(nullptr),
    outputLock(nullptr),
    maxQueueLength(maxQueueLength),
    isRunning(true)
{
    std::cout << "Building " << name << std::endl;
}

void Module::setInputLock(std::shared_ptr<std::mutex> lock) {
    inputLock = lock;
}

void Module::setOutputLock(std::shared_ptr<std::mutex> lock) {
    outputLock = lock;
}

void Module::setInputQueue(std::shared_ptr<TimePriorityQueue> inputQueue) {
    this->inputQueue = inputQueue;
}

void Module::setOutputQueue(std::shared_ptr<TimePriorityQueue> outputQueue) {
    this->outputQueue = outputQueue;
    if (maxQueueLength > 0 && outputQueue) {
        outputQueue->setMaxCount(maxQueueLength);
    }
}

std::shared_ptr<TimePriorityQueue> Module::getOutputQueue(){
    return this->outputQueue;
}

std::shared_ptr<std::mutex> Module::getOutputLock(){
    return this->outputLock;
}

void Module::close() {
    isRunning = false;
}

std::string Module::toString() const {
    return name;
}


// 辅助函数：打印Package内容
template <typename T>
void PrintVector(const std::string &name, const std::vector<T> &vec)
{
    std::cout << name << ": [";
    for (size_t i = 0; i < vec.size(); ++i)
    {
        std::cout << vec[i];
        if (i < vec.size() - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

void PrintPackage(const Package &pkg)
{
    std::cout << "\n=== Package 信息 ===" << std::endl;
    std::cout << "时间戳: " << pkg.time << std::endl;
    std::cout << "UAV ID: " << pkg.uav_id << std::endl;
    std::cout << "相机ID: " << pkg.camera_id << std::endl;
    PrintVector("相机姿态", pkg.camera_pose);
    PrintVector("相机内参", pkg.camera_K);
    PrintVector("相机畸参", pkg.camera_distortion);
    PrintVector("边界框", pkg.Bbox);
    PrintVector("归一化边界框", pkg.norm_Bbox);
    PrintVector("无深度目标UTM", pkg.uav_utm);
    std::cout << "类别ID: " << pkg.class_id << std::endl;
    std::cout << "类别名称: " << pkg.class_name << std::endl;
    std::cout << "跟踪器ID: " << pkg.tracker_id << std::endl;
    std::cout << "UID: " << pkg.uid << std::endl;
    std::cout << "global_id: " << pkg.global_id << std::endl;
    PrintVector("目标UTM位置", pkg.location);
    auto center = pkg.getCenterPoint();
    std::cout << "目标中心点: [" << center[0] << ", " << center[1] << "]" << std::endl;
}