#include "input/input.h"

Input::Input(const std::string &name,
             std::string &&endpoint1,
             std::string &&endpoint2,
             std::string &&topic)
    : name(name),
      isRunning(false),
      poller_(new ZmqPoller(10)),
      subscriber1_(endpoint1, topic, poller_),
      subscriber2_(endpoint2, topic, poller_)
{
    std::cout << "Building " << name << std::endl;

    // 初始化输出队列和锁
    outputQueue = std::make_shared<std::vector<std::shared_ptr<DataPackage>>>();
    outputLock = std::make_shared<std::mutex>();
    messageQueue_ = new threadsafe_queue<std::vector<uint8_t>>();

    // 设置消息回调
    subscriber1_.setMessageHandler(&Input::messageHandler, this);
    subscriber2_.setMessageHandler(&Input::messageHandler, this);
}

Input::~Input()
{
    stop();
    delete poller_;
    delete messageQueue_; // 释放指针
}

// 设置输出队列锁
void Input::setOutputLock(std::shared_ptr<std::mutex> lock)
{
    outputLock = lock;
}

// 设置输出队列
void Input::setOutputQueue(std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> outputQueue)
{
    this->outputQueue = outputQueue;
}

// 获取输出队列锁
std::shared_ptr<std::mutex> Input::getOutputLock()
{
    return outputLock;
}

// 获取输出队列
std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> Input::getOutputQueue()
{
    return outputQueue;
}

// 消息处理回调（静态成员）
void Input::messageHandler(void *param, zmq::message_t &message, std::string &topic)
{
    Input *self = reinterpret_cast<Input *>(param);
    const auto *data = static_cast<uint8_t *>(message.data());

    // 指针方式访问队列
    self->messageQueue_->push(
        std::vector<uint8_t>(data, data + message.size()));
}

// 处理线程
void Input::process()
{
    while (isRunning)
    {
        std::vector<uint8_t> message;

        messageQueue_->wait_and_pop(message);
        try
        {
            auto dataPkg = std::make_shared<DataPackageLandStation>();
            dataPkg->parse_stream_location(message.data(), message.size());

            // 推送至输出队列
            std::lock_guard<std::mutex> lock(*outputLock);
            outputQueue->push_back(dataPkg);
        }
        catch (const std::exception &e)
        {
            std::cerr << "数据解析错误: " << e.what() << std::endl;
        }
    }
}

// 启动线程
void Input::run()
{
    isRunning = true;
    poller_->start();
    thread_ = std::thread(&Input::process, this);
}

// 停止线程
void Input::stop()
{
    if (isRunning)
    {
        isRunning = false;
        poller_->stop();
        join();
    }
}

// 等待线程结束
void Input::join()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}
