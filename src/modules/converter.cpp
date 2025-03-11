#include "modules/converter.h"
#include <stdexcept>
#include <cmath>

PackageConverter::PackageConverter(const std::string& name, std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>>& input_queue,
                                std::shared_ptr<TimePriorityQueue>& output_queue,
                                std::shared_ptr<std::mutex> input_lock,
                                std::shared_ptr<std::mutex> output_lock):
    name(name)
{
    std::cout << "Building " << name << std::endl;
    setInputLock(input_lock);
    setOutputLock(output_lock);
    setInputQueue(input_queue);
    setOutputQueue(output_queue);
}

PackageConverter::~PackageConverter() {
    stop();
}

void PackageConverter::setInputLock(std::shared_ptr<std::mutex> lock){this->inputLock = lock;}

void PackageConverter::setOutputLock(std::shared_ptr<std::mutex> lock){this->outputLock = lock;}

void PackageConverter::setInputQueue(std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> inputQueue){this->inputQueue = inputQueue;}

void PackageConverter::setOutputQueue(std::shared_ptr<TimePriorityQueue> outputQueue){this->outputQueue = outputQueue;}

std::vector<double> PackageConverter::GetCameraIntrinsics(int camera_type) {
    switch (camera_type) {
        case 0: // 电视相机
            return {1000.0, 1000.0, 320.0, 240.0}; // fx, fy, cx, cy
        case 1: // 红外相机
            return {1000.0, 1000.0, 320.0, 240.0};
        case 2: // 微光相机
            return {1000.0, 1000.0, 320.0, 240.0};
        default:
            throw std::runtime_error("未知的相机类型");
    }
}

std::vector<double> PackageConverter::GetCameraDistortion(int camera_type) {
    // 示例畸变参数，实际使用时需要根据相机标定结果设置
    return {0.01, -0.02, 0.001, 0.002, 0.005}; // k1, k2, p1, p2, k3
}

std::vector<double> PackageConverter::PoseToUtm(const std::vector<double>& camera_pose, const std::vector<double>& norm_Bbox, const std::vector<double>& camera_K) {
    // 这里需要实现实际的坐标转换逻辑
    // 步骤1: 从归一化BBox获取中心点
    double cx = norm_Bbox[0] + norm_Bbox[2]/2;
    double cy = norm_Bbox[1] + norm_Bbox[3]/2;
    
    // 步骤2: 使用相机参数反投影到3D空间
    // （此处应包含完整的相机模型计算，示例使用简单线性变换）
    double world_x = cx * camera_K[0] + camera_K[2];
    double world_y = cy * camera_K[1] + camera_K[3];
    
    // 步骤3: 坐标系转换到UTM（示例简单转换）
    return {
        camera_pose[3] + world_x * 0.1,  // X坐标转换
        camera_pose[4] + world_y * 0.1,  // Y坐标转换
        camera_pose[5] - 50.0            // 假设高度下降50米
    };
}

std::string PackageConverter::GetClassName(int class_id) {
    // 这里可以实现实际的类别映射
    return "class_" + std::to_string(class_id);
}

std::vector<double> PackageConverter::NormalizeBbox(const Bbox& bbox, int camera_type) {
    int img_width = img_size[camera_type][0];
    int img_height = img_size[camera_type][1];
    
    return {
        static_cast<double>(bbox.x) / img_width,
        static_cast<double>(bbox.y) / img_height,
        static_cast<double>(bbox.w) / img_width,
        static_cast<double>(bbox.h) / img_height
    };
}

Package PackageConverter::ConvertSingleObject(const std::shared_ptr<DataPackage> data_pkg, const ObjectInfo& obj) {
    Package pkg(static_cast<time_t>(data_pkg->get_timestamp()));
    // 设置同步包
    pkg.dp = data_pkg;
    
    // 设置基本信息
    pkg.uav_id = std::to_string(data_pkg->get_uav_id());
    pkg.camera_id = data_pkg->get_camera_type();
    
    // 设置相机姿态
    CameraMatrix cm = data_pkg->get_camera_matrix();
    pkg.camera_pose = {
        cm.member.yaw,
        cm.member.pitch,
        cm.member.roll,
        cm.member.x,
        cm.member.y,
        cm.member.z
    };
    
    // 设置相机参数
    pkg.camera_K = GetCameraIntrinsics(pkg.camera_id);
    pkg.camera_distortion = GetCameraDistortion(pkg.camera_id);
    
    // 设置边界框
    pkg.Bbox = {
        static_cast<int>(obj.rect.x),
        static_cast<int>(obj.rect.y),
        static_cast<int>(obj.rect.w),
        static_cast<int>(obj.rect.h)
    };
    
    // 设置归一化边界框
    pkg.norm_Bbox = NormalizeBbox(obj.rect, pkg.camera_id);
    
    // 设置目标信息
    pkg.class_id = obj.label;
    pkg.class_name = GetClassName(obj.label);
    pkg.tracker_id = obj.tracker_id;
    
    // 设置UTM坐标
    pkg.uav_utm = PoseToUtm(pkg.camera_pose, pkg.norm_Bbox, pkg.camera_K);
    
    // 设置其他信息
    pkg.uid = obj.uid;
    
    // 设置默认边界框类型
    pkg.setBboxType("xywh");
    
    return pkg;
}

std::vector<Package> PackageConverter::ConvertToPackages(const std::shared_ptr<DataPackage>& data_pkg) {
    std::vector<Package> packages;
    
    // 获取所有目标信息
    auto objects = data_pkg->get_object_info();
    
    // 为每个目标创建一个Package
    for (const auto& obj : objects) {
        packages.push_back(ConvertSingleObject(data_pkg, obj));
    }
    
    return packages;
}

void PackageConverter::process(){
    while (isRunning) {
        std::shared_ptr<DataPackage> data_pkg = nullptr;
        
        // 获取数据包并立即释放锁
        {
            std::lock_guard<std::mutex> guard(*inputLock);
            if (!inputQueue->empty()) {
                data_pkg = inputQueue->front();
                inputQueue->erase(inputQueue->begin());
            } else {
                // 队列为空时短暂休眠避免CPU空转
                inputLock->unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }

        // 处理数据包
        std::vector<Package> packages = ConvertToPackages(data_pkg);

        // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== converter ====================" << std::endl;
        // std::cout << "时间戳 " << data_pkg->get_timestamp() << " 的数据包处理后得到 " << packages.size() << " 个数据包" << std::endl;
        
        // 输出处理后的数据
        outputLock->lock();
        for (const auto& package : packages) {
            // 打印每个包的信息
            // PrintPackage(package);
            
            // 等待输出队列有空间
            while (outputQueue->isFull()) {
                outputLock->unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                outputLock->lock();
            }

            // 推送数据
            outputQueue->push(package);
        }
        outputLock->unlock();
    }
}


void PackageConverter::run(){
    isRunning = true;
    thread_ = std::thread(&PackageConverter::process, this);
}

void PackageConverter::stop() {
    if (isRunning) {
        isRunning = false;
        join();
    }
}

void PackageConverter::join(){
    if (thread_.joinable()) {
        thread_.join();
    }
}