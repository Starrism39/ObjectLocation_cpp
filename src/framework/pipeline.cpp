#include"framework/pipeline.h"

Pipeline::Pipeline(const std::vector<std::vector<Module*>>& raw_modules, std::shared_ptr<TimePriorityQueue> inputQueue, std::shared_ptr<std::mutex> inputLock) {
    setInputLock(inputLock);
    setInputQueue(inputQueue);
    // 将原始指针转换为shared_ptr
    modules.resize(raw_modules.size());
    for (size_t i = 0; i < raw_modules.size(); ++i) {
        modules[i].resize(raw_modules[i].size());
        for (size_t j = 0; j < raw_modules[i].size(); ++j) {
            modules[i][j] = std::shared_ptr<Module>(raw_modules[i][j]);
        }
    }
    build();
}

void Pipeline::setInputLock(std::shared_ptr<std::mutex> lock){this->inputLock = lock;}

void Pipeline::setInputQueue(std::shared_ptr<TimePriorityQueue> inputQueue){this->inputQueue = inputQueue;}


// 正常流程
// void Pipeline::build() {
//     for (size_t stages = 0; stages < modules.size(); ++stages) {
//         queue_list.push_back(std::make_shared<TimePriorityQueue>());
//         lock_list.push_back(std::make_shared<std::mutex>());
        
//         for (size_t m = 0; m < modules[stages].size(); ++m) {
//             modules[stages][m]->setOutputQueue(queue_list[stages]);
//             modules[stages][m]->setOutputLock(lock_list[stages]);
            
//             if (stages != 0) {
//                 modules[stages][m]->setInputQueue(queue_list[stages - 1]);
//                 modules[stages][m]->setInputLock(lock_list[stages - 1]);
//             }
            
//             thread_pool.emplace_back([module = modules[stages][m]]() {
//                 module->run();
//             });
//         }
//     }
// }

// 输入来源不在框架时（也就是需要借助外来数据包）：
void Pipeline::build() {
    // 创建初始输入队列（首阶段之前的队列）
    queue_list.push_back(inputQueue);
    lock_list.push_back(inputLock);
    
    for (size_t stage = 0; stage < modules.size(); ++stage) {
        // 只有不是最后一个阶段时才创建输出队列
        if (stage < modules.size() - 1) {
            queue_list.push_back(std::make_shared<TimePriorityQueue>());
            lock_list.push_back(std::make_shared<std::mutex>());
        }
        
        for (size_t m = 0; m < modules[stage].size(); ++m) {
            auto& module = modules[stage][m];
            // 输入队列为前一阶段的输出队列
            module->setInputQueue(queue_list[stage]);
            module->setInputLock(lock_list[stage]);
            
            // 只有不是最后一个阶段时才设置输出队列
            if (stage < modules.size() - 1) {
                module->setOutputQueue(queue_list[stage + 1]);
                module->setOutputLock(lock_list[stage + 1]);
            }
            
            thread_pool.emplace_back([module]() {
                module->run();
            });
        }
    }
}

void Pipeline::join() {
    for (auto& thread : thread_pool) {
        thread.join();
    }
    std::cout << "all join" << std::endl;
}
