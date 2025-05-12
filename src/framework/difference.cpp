#include "framework/difference.h"
#include <thread>
#include <chrono>

Difference::Difference(const std::string &name, double time_slice, size_t maxQueueLength)
    : Module(name, maxQueueLength), timeSlice(time_slice) {}

void Difference::run()
{
    while (isRunning)
    {
        // std::cout << "running RTK_difference" << std::endl;
        inputLock->lock();
        if (inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1)
        { 
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        std::vector<Package> packages = inputQueue->getTimeSlice(timeSlice);
        inputLock->unlock();

        if (packages.empty())
            continue;

        std::vector<Package> results = process(packages);

        // // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== RTK difference ====================" << std::endl;
        // std::cout << "RTK difference处理后得到 " << results.size() << " 个数据包" << std::endl;

        for (const auto &package : results)
        {
            while (outputQueue->isFull())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            outputLock->lock();
            // // 打印每个包的信息
            // PrintPackage(package);
            outputQueue->push(package);
            outputLock->unlock();
        }
    }
}