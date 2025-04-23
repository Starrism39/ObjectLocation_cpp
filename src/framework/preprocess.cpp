#include "framework/preprocess.h"
#include <thread>
#include <chrono>

PreProcess::PreProcess(const std::string &name, double time_slice, size_t maxQueueLength)
    : Module(name, maxQueueLength), timeSlice(time_slice)
{
    if (maxQueueLength > 0 && outputQueue)
    {
        outputQueue->setMaxCount(maxQueueLength);
    }
}

void PreProcess::run()
{
    while (isRunning)
    {
        // std::cout << "running time_filter" << std::endl;
        // 获取输入锁
        inputLock->lock();

        // 检查输入条件
        if (inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice * 2)
        {
            // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 获取时间片段的数据
        std::vector<Package> packages = inputQueue->getTimeSlice(timeSlice);
        inputLock->unlock();

        // 检查包是否为空
        if (packages.empty())
        {
            continue;
        }

        // 处理数据
        packages = process(packages);

        // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== time filter ====================" << std::endl;
        // std::cout << "time fliter处理后得到 " << packages.size() << " 个数据包" << std::endl;

        // 输出处理后的数据
        for (const auto &package : packages)
        {
            // // 打印每个包的信息
            // PrintPackage(package);

            // 等待输出队列有空间
            while (outputQueue->isFull())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // 获取输出锁并推送数据
            outputLock->lock();
            outputQueue->push(package);
            outputLock->unlock();
        }
    }
}