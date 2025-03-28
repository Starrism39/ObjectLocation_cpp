#include "framework/module.h"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <sstream>

// Package实现
Package::Package(time_t time) : time(time),
                                camera_id(-1),
                                prob(0.0),
                                class_id(-1),
                                tracker_id(-1),
                                uid(-1),
                                delim_flag(0),
                                global_id(-1),
                                local_id(-1),
                                bbox_type("xywh") {}

std::vector<double> Package::getCenterPoint() const
{
    if (Bbox.size() < 4)
    {
        throw std::runtime_error("Bbox未正确初始化");
    }

    std::vector<double> center(2);
    if (bbox_type == "cxcywh")
    {
        center[0] = static_cast<double>(Bbox[0]);
        center[1] = static_cast<double>(Bbox[1] + Bbox[3] / 2.0);
    }
    else if (bbox_type == "xyxy")
    {
        center[0] = static_cast<double>((Bbox[0] + Bbox[2]) / 2.0);
        center[1] = static_cast<double>((Bbox[1] + Bbox[3]) / 2.0);
    }
    else if (bbox_type == "xywh")
    {
        center[0] = static_cast<double>(Bbox[0] + Bbox[2] / 2.0);
        center[1] = static_cast<double>(Bbox[1] + Bbox[3] / 2.0);
    }
    else
    {
        throw std::runtime_error("bbox_type必须是 cxcywh、xyxy 或 xywh");
    }
    return center;
}

Package Package::copy() const
{
    return Package(*this); // 返回对象的副本
}

void Package::setBboxType(const std::string &type)
{
    if (type != "cxcywh" && type != "xyxy" && type != "xywh")
    {
        throw std::runtime_error("bbox_type必须是 cxcywh、xyxy 或 xywh");
    }
    bbox_type = type;
}

std::string Package::toString() const
{
    return "time:" + std::to_string(time);
}

Module::Module(const std::string &name, size_t maxQueueLength)
    : name(name),
      inputQueue(nullptr),
      outputQueue(nullptr),
      inputLock(nullptr),
      outputLock(nullptr),
      maxQueueLength(maxQueueLength),
      isRunning(true)
{
    std::cout << "Building " << name << std::endl;
}

void Module::setInputLock(std::shared_ptr<std::mutex> lock)
{
    inputLock = lock;
}

void Module::setOutputLock(std::shared_ptr<std::mutex> lock)
{
    outputLock = lock;
}

void Module::setInputQueue(std::shared_ptr<TimePriorityQueue<Package>> inputQueue)
{
    this->inputQueue = inputQueue;
}

void Module::setOutputQueue(std::shared_ptr<TimePriorityQueue<Package>> outputQueue)
{
    this->outputQueue = outputQueue;
    if (maxQueueLength > 0 && outputQueue)
    {
        outputQueue->setMaxCount(maxQueueLength);
    }
}

std::shared_ptr<TimePriorityQueue<Package>> Module::getOutputQueue()
{
    return this->outputQueue;
}

std::shared_ptr<std::mutex> Module::getOutputLock()
{
    return this->outputLock;
}

void Module::close()
{
    isRunning = false;
}

std::string Module::toString() const
{
    return name;
}

// 打印信息
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

std::string type2str(int type)
{
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }
    r += "C";
    r += (chans + '0');
    return r;
}

void printOutPackage(const OutPackage &package) {
    // 输出时间信息
    std::cout << "=== Package Contents ===\n";
    std::cout << "Timestamp: " << package.time << "\n";
    std::cout << "Timeslice: " << std::fixed << std::setprecision(3) 
              << package.time_slice << "s\n\n";

    // 输出无人机位姿
    std::cout << "UAV Poses (" << package.uav_pose.size() << "):\n";
    for (const auto &[uav_id, pose] : package.uav_pose) {
        std::cout << "  UAV " << static_cast<int>(uav_id) << " pose: [";
        for (size_t j = 0; j < pose.size(); ++j) {
            if (j > 0) std::cout << ", ";
            std::cout << std::fixed << std::setprecision(3) << pose[j];
        }
        std::cout << "]\n";
    }
    std::cout << "\n";

    // 输出物体信息
    std::cout << "Detected Objects (" << package.objs.size() << "):\n";
    for (size_t i = 0; i < package.objs.size(); ++i) {
        const Object &obj = package.objs[i];
        std::cout << "  Object #" << i+1 << "\n";
        std::cout << "    Global ID: " << obj.global_id << "\n";
        
        // 位置信息
        std::cout << "    Location: [";
        for (size_t j = 0; j < obj.location.size(); ++j) {
            if (j > 0) std::cout << ", ";
            std::cout << std::fixed << std::setprecision(3) << obj.location[j];
        }
        std::cout << "]\n";
        
        // 图像信息
        std::cout << "    UAV Images (" << obj.uav_img.size() << "):\n";
        for (const auto &[uav_id, mat] : obj.uav_img) {
            std::cout << "      UAV " << static_cast<int>(uav_id) << ": "
                      << mat.cols << "x" << mat.rows << " "
                      << type2str(mat.type()) 
                      << " (channels: " << mat.channels() << ")\n";
        }
        std::cout << "\n";
    }
    std::cout << "=== End of Package ===\n\n";
}