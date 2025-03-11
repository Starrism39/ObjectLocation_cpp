#include "framework/filter.h"
#include <thread>
#include <chrono>

Filter::Filter(const std::string& name, double time_slice, size_t maxQueueLength)
    :Module(name, maxQueueLength), timeSlice(time_slice){}

void Filter::run(){
    while(isRunning){
        // std::cout << "running spatial_filter" << std::endl;
        inputLock->lock();
        if(inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1){
            // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            inputLock->unlock();
            continue;
        }
        std::vector<Package> packages = inputQueue->getTimeSlice(timeSlice);
        inputLock->unlock();

        if(packages.empty()) continue;

        packages = process(packages);

        // 打印处理结果
        std::cout << std::string(3, '\n');
        std::cout << "==================== spatial filter ====================" << std::endl;
        std::cout << "spatial fliter处理后得到 " << packages.size() << " 个数据包" << std::endl;

        for(const auto& package : packages){
            while(outputQueue->isFull()){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            outputLock->lock();
            PrintPackage(package);
            outputQueue->push(package);
            outputLock->unlock();
        }
    }
}