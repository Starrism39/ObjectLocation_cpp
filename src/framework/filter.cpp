#include "framework/filter.h"
#include <thread>
#include <chrono>

Filter::Filter(const std::string &name, double time_slice, int parallel, size_t maxQueueLength)
    : Module(name, maxQueueLength), timeSlice(time_slice), paralle_nums(parallel) {}

void Filter::run()
{
    while (isRunning)
    {
        // std::cout << "running spatial_filter" << std::endl;
        inputLock->lock();
        if (inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1 || inputQueue->size() < paralle_nums + 1)
        { // 最后一个判断条件是为了缓解多线程导致的数据乱序问题
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        std::vector<Package> packages = inputQueue->getTimeSlice(timeSlice);
        inputLock->unlock();

        if (packages.empty())
            continue;

        packages = process(packages);

        // // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== spatial filter ====================" << std::endl;
        // std::cout << "spatial fliter处理后得到 " << packages.size() << " 个数据包" << std::endl;

        for (const auto &package : packages)
        {
            while (outputQueue->isFull())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            outputLock->lock();
            // // 打印每个包的信息
            // PrintPackage(package);
            outputQueue->push(package);
            outputLock->unlock();
        }
    }
}