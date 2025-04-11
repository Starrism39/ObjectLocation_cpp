#include <iostream>
#include <zmq.hpp>
#include <string>
#include <thread>
#include <vector>
#include <random>
#include <chrono>
#include <mutex>
#include <iomanip>
#include <filesystem>
#include <functional>

using MessageHandler = std::function<void(void *, zmq::message_t &, std::string &)>;
using Milliseconds = int;

class ZeroMqPublisher
{
public:
    ZeroMqPublisher(std::string &endpoint, std::string topic)
        : endpoint_(endpoint), topic_(topic)
    {
        context_ = new zmq::context_t(1);
        operation_ = new zmq::socket_t(*context_, zmq::socket_type::pub);
        operation_->bind(endpoint);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "[ZeroMqPublisher] bind to " << endpoint.c_str() << std::endl;
    };

    ~ZeroMqPublisher()
    {
        if (operation_ != nullptr)
        {
            delete operation_;
            operation_ = nullptr;
        }
        if (context_ != nullptr)
        {
            delete context_;
            context_ = nullptr;
        }
    };
    void publish(uint8_t *data_buffer, size_t send_len);

private:
    std::string endpoint_;

    std::string topic_;
    zmq::socket_t *operation_ = nullptr;
    zmq::context_t *context_ = nullptr;
};

/**
 * @class ZmqPoller
 * @brief ZeroMQ 套接字轮询器，用于异步处理多个 ZMQ 套接字的消息
 *
 * 该类实现了一个轮询机制，可以同时监听多个 ZeroMQ 套接字，
 * 当有消息到达时调用相应的回调函数进行处理。
 * 轮询操作在单独的线程中进行，以避免阻塞主线程。
 */
class ZmqPoller
{
public:
    /**
     * @brief 构造函数
     * @param poll_timeout 轮询超时时间(毫秒)
     *                     较小的值提供更快的响应时间但增加 CPU 使用率
     *                     较大的值降低 CPU 使用率但可能增加消息处理延迟
     */
    ZmqPoller(Milliseconds poll_timeout) : poll_timeout_(poll_timeout), running_(false) {};
    ~ZmqPoller()
    {
        stop();
    };

    void addSocket(zmq::socket_t *socket,
                   MessageHandler handler,
                   void *handler_param,
                   std::string &topic);
    void start();
    void stop();

private:
    struct Socket
    {
        zmq::socket_t *socket;
        MessageHandler handler;
        void *handler_param;
        std::string topic;
    };

    void run();
    std::vector<Socket> sockets_;
    std::thread poller_thread_;
    int poll_timeout_;
    bool running_;
};

/**
 * @class ZeroMqSubscriber
 * @brief ZeroMQ 订阅者类，用于接收消息并处理
 * @param endpoints 订阅的端点列表
 * @param topic 订阅的主题  
 * @param poll_timeout 轮询超时时间(毫秒)
 * 该类实现了一个 ZeroMQ 订阅者，可以连接到指定的端点，
 * 订阅特定主题的消息，并在接收到消息时调用相应的回调函数进行处理。
 */
class ZeroMqSubscriber : public ZmqPoller
{ 
public:
    ZeroMqSubscriber(std::vector<std::string> &endpoints, std::string topic, Milliseconds poll_timeout)
        : ZmqPoller(poll_timeout), topic_(topic)
    {
        context_ = new zmq::context_t(1);
        operation_ = new zmq::socket_t(*context_, zmq::socket_type::sub);
        for(int i =0;i<endpoints.size();i++)
        {
            operation_->connect(endpoints[i]);
            operation_->set(zmq::sockopt::subscribe, topic_);
            std::cout << "[ZeroMqSubscriber] connect to " << endpoints[i].c_str() << std::endl;
        }
    };

    ~ZeroMqSubscriber()
    {
        if (operation_ != nullptr)
        {
            delete operation_;
            operation_ = nullptr;
        }
        if (context_ != nullptr)
        {
            delete context_;
            context_ = nullptr;
        }
    };
    void addMessageHandler(MessageHandler handler, void *handler_param);


private:
    std::string topic_;
    zmq::socket_t *operation_ = nullptr;
    zmq::context_t *context_ = nullptr;
};