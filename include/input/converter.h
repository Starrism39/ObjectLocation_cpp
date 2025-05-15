#pragma once

#include "framework/module.h"
#include "framework/package.h"
#include <tuple>
#include <vector>
#include <string>
#include <thread>

std::tuple<double, double, int, double> LLHtoUTM(double lon_deg, double lat_deg, double height);

class PackageConverter
{
public:
    PackageConverter(const std::string &name,
                     std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> input_queue,
                     std::shared_ptr<std::mutex> input_lock,
                     uint8_t uav_id,
                     double del_easting,
                     double del_northing,
                     double del_uav0_height,
                     double del_uav1_height,
                     int max_queue_length = 0);
    ~PackageConverter();

    void setInputLock(std::shared_ptr<std::mutex> lock);
    void setOutputLock(std::shared_ptr<std::mutex> lock);
    void setInputQueue(std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> inputQueue);
    void setOutputQueue(std::shared_ptr<TimePriorityQueue<Package>> outputQueue);
    std::shared_ptr<std::mutex> getOutputLock();
    std::shared_ptr<TimePriorityQueue<Package>> getOutputQueue();

    void run();
    void stop();
    void join();

    // 将DataPackage和ObjectInfo转换为Package
    std::vector<Package> ConvertToPackages(const std::shared_ptr<DataPackage> &data_pkg);


private:
    // 处理线程
    void process();


    // 将单个ObjectInfo转换为Package
    Package ConvertSingleObject(const std::shared_ptr<DataPackage> data_pkg, const ObjectInfo &obj);

    // 相机内参设置
    std::vector<double> GetCameraIntrinsics(std::string camera_type);

    // 相机畸变参数设置
    std::vector<double> GetCameraDistortion(std::string camera_type);

    // 从相机姿态获取UTM坐标
    std::vector<double> PoseToUtm(const std::vector<double> &camera_pose, const std::vector<double> &norm_Bbox, const std::vector<double> &camera_K);

    // 获取类别名称
    std::string GetClassName(int class_id);

    // 计算归一化边界框
    std::vector<double> NormalizeBbox(const Bbox &bbox, int camera_type);

protected:
    std::string name;
    std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> inputQueue;
    std::shared_ptr<TimePriorityQueue<Package>> outputQueue;
    std::shared_ptr<std::mutex> inputLock;
    std::shared_ptr<std::mutex> outputLock;
    uint8_t uav_id_;
    double del_easting;
    double del_northing;
    double del_uav0_height;
    double del_uav1_height;
    std::atomic<bool> isRunning;
    std::thread thread_;
};
