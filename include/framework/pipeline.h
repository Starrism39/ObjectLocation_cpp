#include <vector>
#include <thread>
#include <mutex>
#include <memory>
#include "framework/module.h"


class Pipeline {
private:
    std::vector<std::vector<std::shared_ptr<Module>>> modules;
    std::vector<std::shared_ptr<TimePriorityQueue>> queue_list;
    std::vector<std::thread> thread_pool;
    std::vector<std::shared_ptr<std::mutex>> lock_list;

    void build();

public:
    Pipeline(const std::vector<std::vector<Module*>>& raw_modules, std::shared_ptr<TimePriorityQueue> inputQueue, std::shared_ptr<std::mutex> inputLock);

    void setInputLock(std::shared_ptr<std::mutex> lock);
    void setInputQueue(std::shared_ptr<TimePriorityQueue> inputQueue);
    
    void join();

protected:
    std::shared_ptr<TimePriorityQueue> inputQueue;
    std::shared_ptr<std::mutex> inputLock;
};
