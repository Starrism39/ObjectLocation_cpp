#include "modules/PkgArrange.h"
#include <thread>

PkgArrange::PkgArrange(int max_queue_length): Arrange::Arrange("PkgArrange", max_queue_length),
                                              max_queue_length(max_queue_length) {}

void PkgArrange::process() {
    // 阶段1：快速检查是否有工作可做（不加锁）
    if (inputQueue->isEmpty()) {
        return;
    }

    // 阶段2：获取输入队列快照（最小化锁时间）
    std::list<Package> snapshot;
    {
        std::lock_guard<std::mutex> input_guard(*inputLock);
        if (inputQueue->isEmpty()) {
            return; // 双重检查
        }
        snapshot.assign(inputQueue->getQueue().begin(), 
                            inputQueue->getQueue().end());
    }

    // 阶段3：处理快照数据（无锁操作）
    auto rit = snapshot.rbegin();
    if (rit == snapshot.rend()) {
        return;
    }

    const auto base_time = rit->time;
    const auto base_uav_id = rit->uav_id;
    const int expected_num = rit->num;

    int actual_count = std::count_if(snapshot.rbegin(), snapshot.rend(),
        [base_time, base_uav_id](const Package& pkg) {
            return pkg.time == base_time && pkg.uav_id == base_uav_id;
        });

    if (actual_count < expected_num) {
        return;
    }

    // 收集完整分组
    std::vector<Package> group;
    group.reserve(expected_num);
    for (auto it = snapshot.rbegin(); it != snapshot.rend() && group.size() < expected_num; ++it) {
        if (it->time == base_time && it->uav_id == base_uav_id) {
            group.push_back(*it);
        } else if (it->time != base_time) {
            break;
        }
    }

    // 检查是否真的收集到足够数量（防御性编程）
    if (group.size() != expected_num) {
        return;
    }

    // 阶段4：从原队列中移除已处理元素（加锁）
    {
        std::lock_guard<std::mutex> input_guard(*inputLock);
        // 由于队列可能已改变，需要重新定位要删除的元素
        auto& queue = inputQueue->getQueue();
        int removed = 0;
        for (auto it = queue.rbegin(); it != queue.rend() && removed < expected_num; ) {
            if (it->time == base_time && it->uav_id == base_uav_id) {
                // 转换为正向迭代器进行删除
                auto to_erase = (++it).base();
                it = std::reverse_iterator(queue.erase(to_erase));
                ++removed;
            } else if (it->time != base_time) {
                break;
            } else {
                ++it;
            }
        }
    }

    // // 打印处理结果
    // std::cout << std::string(3, '\n');
    // std::cout << "==================== PkgArrange ====================" << std::endl;

    // 阶段5：输出处理结果（优化输出锁）
    if (!group.empty()) {
        std::unique_lock<std::mutex> output_guard(*outputLock);
        for (auto it = group.rbegin(); it != group.rend(); ++it) {
            while (outputQueue->isFull()) {
                outputLock->unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                outputLock->lock();
            }
            // // 打印每个包的信息
            // PrintPackage(*it);
            outputQueue->push(*it);
        }
    }
}