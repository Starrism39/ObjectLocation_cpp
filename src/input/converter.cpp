#include "input/converter.h"
#include <stdexcept>
#include <cmath>

std::tuple<double, double, int, double> LLHtoUTM(double lon_deg, double lat_deg, double height){
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double k0 = 0.9996;
    const double E2 = 2 * f - f * f;
    const double e_prime_squared = E2 / (1 - E2);
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
   
    int zone = static_cast<int>((lon_deg + 180.0) / 6.0) + 1;
    if(lat_deg >= 56.0 && lat_deg < 64.0 && lon_deg >= 3.0 && lon_deg < 12.0) zone = 32;
    if(lat_deg >= 72.0 && lat_deg < 84.0){
        if(lon_deg >= 0.0 && lon_deg < 9.0) zone = 31;
        else if(lon_deg >= 9.0 && lon_deg < 21.0) zone = 33;
        else if(lon_deg >= 21.0 && lon_deg <33.0) zone = 35;
        else if(lon_deg >= 33.0 && lon_deg < 42.0) zone = 37;
   }

    double lon0 = (zone - 1) * 6.0 - 180.0 + 3.0;
    lon0 = lon0 * M_PI / 180.0;

    double dLon = lon - lon0;

    double N = a / sqrt(1 - E2 * pow(sin(lat), 2));

    double T = pow(tan(lat), 2);
    double C = e_prime_squared * pow(cos(lat), 2);
    double A = dLon * cos(lat);


    double M = a * ((1 - E2 / 4 - 3 * pow(E2, 2) / 64 - 5 * pow(E2, 3) / 256) * lat - 
                    (3 * E2 / 8 + 3 * pow(E2, 2) / 32 + 45 * pow(E2, 3) / 1024) * sin(2 * lat) +
                    (15 * pow(E2, 2)/256 + 45*pow(E2, 3)/1024) * sin(4*lat) - (35*pow(E2, 3)/3072) * sin(6*lat));
    double easting = k0 * N * (A + (1 - T + C) * pow(A, 3)/6 + (5 - 18*T + pow(T, 2) + 72*C -58*e_prime_squared) * pow(A, 5)/120) + 500000.0;
    double northing = k0 * (M + N * tan(lat) * (pow(A, 2)/2 + (5 - T + 9*C + 4*pow(C, 2)) * pow(A, 4)/24 + (61 - 58*T + pow(T, 2) + 600*C -330*e_prime_squared) * pow(A, 6)/720));
    
    if (lat < 0) northing += 10000000.0;
    
    return std::make_tuple(easting, northing, zone, height);

}

Package PackageConverter::Simulation(const std::shared_ptr<DataPackage> data_pkg)
{
    Package pkg(static_cast<time_t>(data_pkg->get_timestamp()));
    // 设置同步包
    pkg.dp = data_pkg;

    // 设置基本信息
    pkg.uav_id = std::to_string(data_pkg->get_uav_id());
    pkg.camera_id = data_pkg->get_camera_type();

    // 设置相机姿态
    CameraMatrix cm = data_pkg->get_camera_matrix();
    auto [easting, northing, zone, height] = LLHtoUTM(cm.member.y, cm.member.x, cm.member.z);
    pkg.camera_pose = {
        cm.member.yaw,
        cm.member.pitch,
        cm.member.roll,
        easting,
        northing,
        height};

    // pkg.camera_pose = {
    //     cm.member.yaw,
    //     cm.member.pitch,
    //     cm.member.roll,
    //     cm.member.x,
    //     cm.member.y,
    //     cm.member.z};

    // 设置相机参数
    pkg.camera_K = GetCameraIntrinsics(pkg.camera_id);
    pkg.camera_distortion = GetCameraDistortion(pkg.camera_id);

    // 设置边界框（中心：(640, 540)）
    pkg.Bbox = {
        static_cast<int>(959),
        static_cast<int>(539),
        static_cast<int>(2),
        static_cast<int>(2)};

    Bbox bbox = {static_cast<uint16_t>(pkg.Bbox[0]), 
             static_cast<uint16_t>(pkg.Bbox[1]), 
             static_cast<uint16_t>(pkg.Bbox[2]), 
             static_cast<uint16_t>(pkg.Bbox[3])};

    // std::cout << "bbox: x=" << pkg.Bbox[0]
    //           << ", y=" << pkg.Bbox[1]
    //           << ", w=" << pkg.Bbox[2]
    //           << ", h=" << pkg.Bbox[3] << std::endl;

    pkg.prob = 0.50;

    // 设置归一化边界框
    pkg.norm_Bbox = NormalizeBbox(bbox, pkg.camera_id);

    // 设置目标信息
    pkg.class_id = 0;
    pkg.class_name = GetClassName(0);
    pkg.tracker_id = 0;

    // 设置UTM坐标
    pkg.uav_utm = PoseToUtm(pkg.camera_pose, pkg.norm_Bbox, pkg.camera_K);

    // 设置其他信息
    pkg.uid = 0;

    // 设置默认边界框类型
    pkg.setBboxType("xywh");

    return pkg;
}

PackageConverter::PackageConverter(const std::string &name,
                                   std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> input_queue,
                                   std::shared_ptr<std::mutex> input_lock,
                                   int max_queue_length) : name(name),
                                                           inputQueue(std::move(input_queue)),
                                                           inputLock(std::move(input_lock))
{
    std::cout << "\nBuilding " << name << std::endl;
    this->outputLock = std::make_shared<std::mutex>();
    this->outputQueue = std::make_shared<TimePriorityQueue<Package>>();
    if (max_queue_length > 0 && outputQueue)
    {
        this->outputQueue->setMaxCount(max_queue_length);
    }
}

PackageConverter::~PackageConverter()
{
    stop();
}

void PackageConverter::setInputLock(std::shared_ptr<std::mutex> lock) { this->inputLock = lock; }

void PackageConverter::setOutputLock(std::shared_ptr<std::mutex> lock) { this->outputLock = lock; }

void PackageConverter::setInputQueue(std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> inputQueue) { this->inputQueue = inputQueue; }

void PackageConverter::setOutputQueue(std::shared_ptr<TimePriorityQueue<Package>> outputQueue) { this->outputQueue = outputQueue; }

std::shared_ptr<std::mutex> PackageConverter::getOutputLock() { return this->outputLock; }

std::shared_ptr<TimePriorityQueue<Package>> PackageConverter::getOutputQueue() { return this->outputQueue; }

std::vector<double> PackageConverter::GetCameraIntrinsics(int camera_type)
{
    switch (camera_type)
    {
    case 0:                                    // 电视相机
        return {6509.18070, 6501.34349, 1061.37015, 698.79659}; // fx, fy, cx, cy
    case 1:                                    // 红外相机
        return {1000.0, 1000.0, 320.0, 240.0};
    case 2:                                    // 微光相机
        return {1000.0, 1000.0, 320.0, 240.0};
    default:
        throw std::runtime_error("未知的相机类型");
    }
}

std::vector<double> PackageConverter::GetCameraDistortion(int camera_type)
{
    switch (camera_type)
    {
    case 0:                                    // 电视相机
        return {-0.13074778, -3.96066684, -0.00492515, 0.00253830, 0.0}; // k1, k2, p1, p2, k3
    case 1:                                    // 红外相机
        return {0, 0, 0, 0, 0};
    case 2:                                    // 微光相机
        return {0, 0, 0, 0, 0};
    default:
        throw std::runtime_error("未知的相机类型");
    }
    
}

std::vector<double> PackageConverter::PoseToUtm(const std::vector<double> &camera_pose, const std::vector<double> &norm_Bbox, const std::vector<double> &camera_K)
{
    // 这里需要实现实际的坐标转换逻辑
    // 步骤1: 从归一化BBox获取中心点
    double cx = norm_Bbox[0] + norm_Bbox[2] / 2;
    double cy = norm_Bbox[1] + norm_Bbox[3] / 2;

    // 步骤2: 使用相机参数反投影到3D空间
    // （此处应包含完整的相机模型计算，示例使用简单线性变换）
    double world_x = cx * camera_K[0] + camera_K[2];
    double world_y = cy * camera_K[1] + camera_K[3];

    // 步骤3: 坐标系转换到UTM（示例简单转换）
    return {
        camera_pose[3] + world_x * 0.1, // X坐标转换
        camera_pose[4] + world_y * 0.1, // Y坐标转换
        camera_pose[5] - 50.0           // 假设高度下降50米
    };
}

std::string PackageConverter::GetClassName(int class_id)
{
    // 这里可以实现实际的类别映射
    return "class_" + std::to_string(class_id);
}

std::vector<double> PackageConverter::NormalizeBbox(const Bbox &bbox, int camera_type)
{
    int img_width = img_size[camera_type][0];
    int img_height = img_size[camera_type][1];

    return {
        static_cast<double>(bbox.x) / img_width,
        static_cast<double>(bbox.y) / img_height,
        static_cast<double>(bbox.w) / img_width,
        static_cast<double>(bbox.h) / img_height};
}

Package PackageConverter::ConvertSingleObject(const std::shared_ptr<DataPackage> data_pkg, const ObjectInfo &obj)
{
    Package pkg(static_cast<time_t>(data_pkg->get_timestamp()));
    // 设置同步包
    pkg.dp = data_pkg;

    // 设置基本信息
    pkg.uav_id = std::to_string(data_pkg->get_uav_id());
    pkg.camera_id = data_pkg->get_camera_type();

    // 设置相机姿态
    CameraMatrix cm = data_pkg->get_camera_matrix();
    auto [easting, northing, zone, height] = LLHtoUTM(cm.member.x, cm.member.y, cm.member.z);
    pkg.camera_pose = {
        cm.member.yaw,
        cm.member.pitch,
        cm.member.roll,
        easting,
        northing,
        height};

    // 设置相机参数
    pkg.camera_K = GetCameraIntrinsics(pkg.camera_id);
    pkg.camera_distortion = GetCameraDistortion(pkg.camera_id);

    // 设置边界框
    pkg.Bbox = {
        static_cast<int>(obj.rect.x),
        static_cast<int>(obj.rect.y),
        static_cast<int>(obj.rect.w),
        static_cast<int>(obj.rect.h)};

    // std::cout << "bbox: x=" << pkg.Bbox[0]
    //           << ", y=" << pkg.Bbox[1]
    //           << ", w=" << pkg.Bbox[2]
    //           << ", h=" << pkg.Bbox[3] << std::endl;

    pkg.prob = obj.prob;

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

std::vector<Package> PackageConverter::ConvertToPackages(const std::shared_ptr<DataPackage> &data_pkg)
{
    std::vector<Package> packages;

    // 获取所有目标信息
    auto objects = data_pkg->get_object_info();

    // 模拟图像正中心数据
    packages.push_back(Simulation(data_pkg));

    // // 为每个目标创建一个Package
    // for (const auto &obj : objects)
    // {
    //     packages.push_back(ConvertSingleObject(data_pkg, obj));
    // }

    return packages;
}

void PackageConverter::process()
{
    while (isRunning)
    {
        std::shared_ptr<DataPackage> data_pkg = nullptr;

        // 获取数据包并立即释放锁
        {
            std::lock_guard<std::mutex> guard(*inputLock);
            if (!inputQueue->empty())
            {
                data_pkg = inputQueue->front();
                inputQueue->erase(inputQueue->begin());
            }
            else
            {
                // 队列为空时短暂休眠避免CPU空转
                inputLock->unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }

        // 处理数据包
        std::vector<Package> packages = ConvertToPackages(data_pkg);

        // // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== converter ====================" << std::endl;
        // std::cout << "时间戳 " << data_pkg->get_timestamp() << " 的数据包处理后得到 " << packages.size() << " 个数据包" << std::endl;

        // 输出处理后的数据
        outputLock->lock();
        for (const auto &package : packages)
        {
            // // 打印每个包的信息
            // PrintPackage(package);

            // 等待输出队列有空间
            while (outputQueue->isFull())
            {
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

void PackageConverter::run()
{
    isRunning = true;
    thread_ = std::thread(&PackageConverter::process, this);
}

void PackageConverter::stop()
{
    if (isRunning)
    {
        isRunning = false;
        join();
    }
}

void PackageConverter::join()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}