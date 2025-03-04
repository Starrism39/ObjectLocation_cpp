#include"framework/preprocess.h"
#include <thread>
#include <chrono>

PreProcess::PreProcess(const std::string& name, double time_slice, size_t maxQueueLength)
    : Module(name, maxQueueLength)
    , timeSlice(time_slice)
{
    if (maxQueueLength > 0 && outputQueue) {
        outputQueue->setMaxCount(maxQueueLength);
    }
}

void PreProcess::run() {
    while (isRunning) {
        // std::cout << "running time_filter" << std::endl;
        // 获取输入锁
        inputLock->lock();
        
        // 检查输入条件
        if (inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1) {
            // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            inputLock->unlock();
            continue;
        }

        // 获取时间片段的数据
        std::vector<Package> packages = inputQueue->getTimeSlice(timeSlice);
        inputLock->unlock();

        // 检查包是否为空
        if (packages.empty()) {
            continue;
        }

        // 处理数据
        packages = process(packages);

        // 输出处理后的数据
        for (const auto& package : packages) {
            // 等待输出队列有空间
            while (outputQueue->isFull()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            // 获取输出锁并推送数据
            outputLock->lock();
            outputQueue->push(package);
            outputLock->unlock();
        }
    }
}