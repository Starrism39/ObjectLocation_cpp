#include "modules/PkgArrange.h"
#include <thread>

PkgArrange::PkgArrange(int max_queue_length): Arrange::Arrange("PkgArrange", max_queue_length),
                                              max_queue_length(max_queue_length) {}

void PkgArrange::process()
{
    // 获取输入锁
    inputLock->lock();
    
    if (inputQueue->isEmpty()) {
        inputLock->unlock();
        return;
    }

    // 获取队列引用
    auto& queue = inputQueue->getQueue();

    // 从队尾开始检查（最早时间）
    auto current_rit = queue.rbegin();
    if (current_rit == queue.rend()){
        inputLock->unlock();
        return;
    }

    // 获取基准组标识
    const auto base_group = std::make_pair(current_rit->time, current_rit->uav_id);
    int expected_num = current_rit->num;
    
    // 扫描同组包（最多遍历到不同组为止）
    int actual_count = 0;
    for (auto rit = current_rit; rit != queue.rend(); ++rit) {
        if (rit->time != base_group.first || rit->uav_id != base_group.second) {
            break; // 发现不同组立即终止
        }
        ++actual_count;
    }

    // 判断是否满足条件
    if (actual_count < expected_num) {
        inputLock->unlock();
        return; // 不满足要求
    }

    // 收集完整分组（从队尾开始取expected_num个）
    std::vector<Package> group;
    group.reserve(expected_num);
    
    auto erase_start = std::prev(queue.end(), actual_count); // 定位起始位置
    for (int i = 0; i < expected_num; ++i) {
        group.push_back(std::move(*erase_start));
        erase_start = queue.erase(erase_start);
    }

    inputLock->unlock();

    // 生成输出包
    while(!group.empty()) {
        while (outputQueue->isFull())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        outputLock->lock();
        outputQueue->push(group.back());
        group.pop_back();
        outputLock->unlock();
    }
}