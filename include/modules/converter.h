#pragma once

#include "framework/module.h"
#include "input/package.h"
#include <vector>
#include <string>
#include <thread>

class PackageConverter {
public:
    PackageConverter(const std::string& name, 
        std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>>& input_queue,
        std::shared_ptr<TimePriorityQueue>& output_queue,
        std::shared_ptr<std::mutex> inputLock,
        std::shared_ptr<std::mutex> outputLock);
    ~PackageConverter();


    void setInputLock(std::shared_ptr<std::mutex> lock);
    void setOutputLock(std::shared_ptr<std::mutex> lock);
    void setInputQueue(std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> inputQueue);
    void setOutputQueue(std::shared_ptr<TimePriorityQueue> outputQueue);

    void run();
    void stop();
    void join();

    // 将DataPackage和ObjectInfo转换为Package
    static std::vector<Package> ConvertToPackages(const std::shared_ptr<DataPackage>& data_pkg);

private:
    // 处理线程
    void process();

    // 将单个ObjectInfo转换为Package
    static Package ConvertSingleObject(const std::shared_ptr<DataPackage> data_pkg, const ObjectInfo& obj);
    
    // 相机内参设置
    static std::vector<double> GetCameraIntrinsics(int camera_type);
    
    // 相机畸变参数设置
    static std::vector<double> GetCameraDistortion(int camera_type);
    
    // 从相机姿态获取UTM坐标
    static std::vector<double> PoseToUtm(const std::vector<double>& camera_pose, const std::vector<double>& norm_Bbox, const std::vector<double>& camera_K);
    
    // 获取类别名称
    static std::string GetClassName(int class_id);
    
    // 计算归一化边界框
    static std::vector<double> NormalizeBbox(const Bbox& bbox, int camera_type);

protected:
    std::string name;
    std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> inputQueue;
    std::shared_ptr<TimePriorityQueue> outputQueue;
    std::shared_ptr<std::mutex> inputLock;
    std::shared_ptr<std::mutex> outputLock;
    std::atomic<bool> isRunning;
    std::thread thread_;
};
