#include <cstring>
#include "modules/output.h"
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
    std::cout << "Building " << name << std::endl;
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

void Output::process()
{
    while (isRunning)
    {
        // std::cout << "running fusion" << std::endl;
        inputLock->lock();
        if (inputQueue->isEmpty())
        {
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== output ====================" << std::endl;

        OutPackage package = inputQueue->pop();
        inputLock->unlock();

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
