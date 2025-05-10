#include "input/input.h"

Input::Input(const std::string &name,
             std::vector<std::string> endpoints,
             std::string &&topic)
    : name(name),
      isRunning(false),
      subscriber_(endpoints, topic, 10)
{
    // std::cout << "Building " << name << std::endl;

    // 初始化输出队列和锁
    outputQueue = std::make_shared<std::vector<std::shared_ptr<DataPackage>>>();
    outputLock = std::make_shared<std::mutex>();
    messageQueue_ = new threadsafe_queue<std::vector<uint8_t>>();

    // 设置消息回调
    subscriber_.addMessageHandler(&Input::messageHandler, this);
}

Input::~Input()
{
    stop();
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
    if (message.size() != topic.size())
    {
        self->messageQueue_->push(
            std::vector<uint8_t>(data, data + message.size()));
    }
}

// 处理线程
void Input::process()
{
    while (isRunning)
    {
        std::vector<uint8_t> message;

        messageQueue_->wait_and_pop(message);
        // std::cout << "主线程收到消息大小: " << message.size() << std::endl;
        try
        {
            
            auto dataPkg = std::make_shared<DataPackageLandStation>();
            dataPkg->parse_stream_location((uint8_t *)message.data(), message.size());

            // 推送至输出队列
            std::lock_guard<std::mutex> lock(*outputLock);
            // std::cout << "添加数据包到输入队列，时间戳: " << dataPkg->get_timestamp() << std::endl;
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
    subscriber_.start();
    thread_ = std::thread(&Input::process, this);
}

// 停止线程
void Input::stop()
{
    if (isRunning)
    {
        isRunning = false;
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
