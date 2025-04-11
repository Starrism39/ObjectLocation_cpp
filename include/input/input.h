#include <iostream>
#include <string>
#include <chrono>
#include <atomic>
#include <mutex>
#include <iomanip>

#include "utils/zero_mq_client.h"
#include "utils/package_land_station.h"
#include "utils/threadsafe_queue.h"

class Input
{
public:
    Input(const std::string &name,
          std::string &&endpoint1,
          std::string &&endpoint2,
          std::string &&topic);
    ~Input();

    void setOutputLock(std::shared_ptr<std::mutex> lock);
    void setOutputQueue(std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> outputQueue);
    std::shared_ptr<std::mutex> getOutputLock();
    std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> getOutputQueue();

    void run();
    void stop();
    void join();

private:
    // 处理线程
    void process();

    // ZeroMQ 消息回调（静态成员）
    static void messageHandler(void *param, zmq::message_t &message, std::string &topic);

protected:
    std::string name;
    std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> outputQueue;
    std::shared_ptr<std::mutex> outputLock;
    std::atomic<bool> isRunning;
    std::thread thread_;

    ZmqPoller *poller_;
    ZeroMqSubscriber subscriber1_;
    ZeroMqSubscriber subscriber2_;
    threadsafe_queue<std::vector<uint8_t>>* messageQueue_;
};
