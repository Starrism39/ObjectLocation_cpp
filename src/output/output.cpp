#include <cstring>
#include "output/output.h"
#include "utils/protocol.h"

Output::Output(const std::string &name,
               const std::string &ip,
               int port,
               const std::string &interface,
               std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue,
               std::shared_ptr<std::mutex> inputLock) : name(name),
                                                        inputQueue(std::move(inputQueue)),
                                                        inputLock(std::move(inputLock)),
                                                        ip(ip)
{
    std::cout << "\nBuilding " << name << std::endl;
    y_ = 0.0;
    this->server = std::make_shared<UDPOperation>(ip.c_str(), port, interface.c_str());
    this->server->create_server();
}

Output::~Output()
{
    stop();
}

void Output::setInputLock(std::shared_ptr<std::mutex> lock) { this->inputLock = lock; }

void Output::setInputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue) { this->inputQueue = inputQueue; }

bool Output::serializeOutPackage(const OutPackage &pkg, std::vector<uint8_t> &buffer)
{
    using namespace ProtocolUtils;

    buffer.insert(buffer.end(), {0xEB, 0x22, 0x90, 0x99});

    uint64_t timestamp = pkg.time;
    serialize(buffer, timestamp);

    serialize<uint16_t>(buffer, static_cast<uint16_t>(pkg.time_slice));

    uint8_t poseCount = pkg.uav_pose.size();
    serialize<uint8_t>(buffer, poseCount);

    for (const auto &[uavId, pose] : pkg.uav_pose)
    {
        serialize<uint8_t>(buffer, uavId);
        for (const auto &val : pose)
        {
            serialize<double>(buffer, val);
        }
    }

    uint16_t objCount = pkg.objs.size();
    serialize<uint16_t>(buffer, objCount);

    for (const auto &obj : pkg.objs)
    {
        serialize<uint32_t>(buffer, obj.global_id);
        for (const auto &coord : obj.location)
        {
            serialize<double>(buffer, coord);
        }

        uint8_t imgCount = obj.uav_img.size();
        serialize<uint8_t>(buffer, imgCount);

        for (const auto &[uavId, img] : obj.uav_img)
        {
            serialize<uint8_t>(buffer, uavId);
            serializeMat(buffer, img);
        }
    }

    buffer.insert(buffer.end(), {0xED, 0xDC, 0xCB, 0xBA});
    return true;
}

void Output::output(const OutPackage& pkg){
    std::vector<uint8_t> buffer;
    if (!serializeOutPackage(pkg, buffer)) {
        std::cerr << "Serialization failed!" << std::endl;
        return ;
    }
    sendFragmented(server, buffer, 0xAA55CC33);
}

void Output::processObjectsInCircle(OutPackage& pkg, double center_x, double center_y, double radius){
    for (auto& obj : pkg.objs) {
        // 从global_id提取label并验证
        const uint8_t current_label = static_cast<uint8_t>(obj.global_id & 0xFF);
        if (current_label != 0) continue;

        // 检查位置有效性
        if (obj.location.size() < 2) continue;

        // 计算与圆心的平面距离
        const double dx = obj.location[0] - center_x;
        const double dy = obj.location[1] - center_y;
        const double dist_sq = dx*dx + dy*dy;

        // 判断是否在圆内并更新
        if (dist_sq <= radius*radius) {
            const uint8_t new_label = 4;
            obj.global_id = (obj.global_id & 0xFFFFFF00) | new_label; // 保留高24位
            obj.label = new_label;  // 同步更新结构体字段
            // std::cout << "Object " << obj.global_id << " is in the circle." << std::endl;
        }
    }
}

void Output::processObjectsInRectangle(OutPackage& pkg, 
                                    double min_x, double max_x,
                                    double min_y, double max_y)
{
    for (auto& obj : pkg.objs) {
        // 从global_id提取label并验证
        const uint8_t current_label = static_cast<uint8_t>(obj.global_id & 0xFF);
        if (current_label != 0) continue;

        // 检查位置有效性
        if (obj.location.size() < 2) continue;

        // 获取坐标值
        const double x = obj.location[0];
        const double y = obj.location[1];

        // 判断是否在矩形区域内
        if (x >= min_x && x <= max_x && 
            y >= min_y && y <= max_y) 
        {
            const uint8_t new_label = 4;
            obj.global_id = (obj.global_id & 0xFFFFFF00) | new_label;
            obj.label = new_label;
            // std::cout << "Object " << obj.global_id << " is in the rectangle." << std::endl;
        }
    }
}

void Output::adjustEnmies(OutPackage& pkg){
    for (auto& obj : pkg.objs) {
        if(static_cast<uint8_t>(obj.global_id & 0xFF) == 1){
            y_ = obj.location[1];
        } 
    }
    for (auto& obj : pkg.objs) {
        if(static_cast<uint8_t>(obj.global_id & 0xFF) == 0 && obj.location[1] < y_){
            const uint8_t new_label = 4;
            obj.global_id = (obj.global_id & 0xFFFFFF00) | new_label;
            obj.label = new_label;
        } 
    }
}

void Output::process()
{
    while (isRunning)
    {
        // std::cout << "running output" << std::endl;
        inputLock->lock();
        if (inputQueue->isEmpty())
        {
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 打印处理结果
        std::cout << std::string(3, '\n');
        std::cout << "==================== output ====================" << std::endl;


        OutPackage package = inputQueue->pop();
        inputLock->unlock();

        // processObjectsInCircle(package, 0.0, 0.0, 3000);
        // processObjectsInRectangle(package, -2000, 2000, -2000, 2000);
        adjustEnmies(package);

        // 打印每个包的信息
        printOutPackage(package);

        output(package);

        // std::cout << "send to " << ip << " successful" << std::endl;

    }
}

void Output::run()
{
    isRunning = true;
    thread_ = std::thread(&Output::process, this);
}

void Output::stop()
{
    if (isRunning)
    {
        isRunning = false;
        join();
    }
}

void Output::join()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}
