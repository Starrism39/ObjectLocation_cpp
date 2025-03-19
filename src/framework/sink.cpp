#include "framework/sink.h"
#include <thread>
#include <chrono>

Sink::Sink(const std::string &name, double time_slice, size_t maxQueueLength)
    : Module(name, maxQueueLength), timeSlice(time_slice) {}

void Sink::run()
{
    while (isRunning)
    {

        inputLock->lock();

        if (inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1)
        {
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 获取第一个包确定时间戳和无人机ID
        Package firstPackage = inputQueue->pop();
        time_t currentTime = firstPackage.time;
        std::string currentUavId = firstPackage.uav_id;
        inputLock->unlock();

        std::vector<Package> sameTimeUavPackages;
        sameTimeUavPackages.push_back(firstPackage); // 压入第一个包

        // 检查队列中的下一个包
        while (true)
        {
            inputLock->lock();

            if (inputQueue->isEmpty())
            {
                inputLock->unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            Package nextPackage = inputQueue->peek();

            // 如果下一个包的时间戳或无人机ID不同，则停止收集
            if (nextPackage.time != currentTime || nextPackage.uav_id != currentUavId)
            {
                inputLock->unlock(); 
                break;
            }

            // 时间戳和无人机ID相同，则提取该包并添加到集合中
            sameTimeUavPackages.push_back(inputQueue->pop());
            inputLock->unlock();
        } 

        // 打印处理结果
        std::cout << std::string(3, '\n');
        std::cout << "==================== Sink ====================" << std::endl;

        // 处理收集到的包
        if (!sameTimeUavPackages.empty())
        {
            std::shared_ptr<DataPackage> dp = process(sameTimeUavPackages);
            // 打印输出信息
            printObjectInfo(dp);
        }


    }
}