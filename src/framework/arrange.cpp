#include "framework/arrange.h"
#include <thread>
#include <chrono>

Arrange::Arrange(const std::string &name, size_t maxQueueLength)
    : Module(name, maxQueueLength) {}

void Arrange::run()
{
    while (isRunning)
    {
        // std::cout << "running arrange" << std::endl;

        // 处理包
        process();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    }
}