#include "framework/filter.h"
#include <thread>
#include <chrono>

Filter::Filter(const std::string& name, double time_slice, size_t maxQueueLength)
    :Module(name, maxQueueLength), timeSlice(time_slice){}

void Filter::run(){
    while(isRunning){
        inputLock->lock();
        if(inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1){
            inputLock->unlock();
            continue;
        }
        std::vector<Package> packages = inputQueue->getTimeSlice(timeSlice);
        inputLock->unlock();

        if(packages.empty()) continue;

        packages = process(packages);
        for(const auto& package : packages){
            while(outputQueue->isFull()){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            outputLock->lock();
            outputQueue->push(package);
            outputLock->unlock();
        }
    }
}