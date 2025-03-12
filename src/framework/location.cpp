#include "framework/location.h"
#include <thread>
#include <chrono>

Location::Location(const std::string &name, size_t maxQueueLength)
    : Module(name, maxQueueLength) {}

void Location::run()
{
    while (isRunning)
    {
        // std::cout << "running esti_position" << std::endl;
        // 获取输入锁
        inputLock->lock();

        if (inputQueue->isEmpty())
        {
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 获取包
        Package package = inputQueue->pop();
        inputLock->unlock();

        // 处理包
        process(package);

        // // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== location ====================" << std::endl;

        // 等待输出队列有空间
        while (outputQueue->isFull())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 输出处理后的包
        outputLock->lock();
        // // 打印每个包的信息
        // PrintPackage(package);
        if(!package.location.empty()){
            outputQueue->push(package);
        }
        outputLock->unlock();
    }
}