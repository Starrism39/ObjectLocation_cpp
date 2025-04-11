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

void IpcPath_check(std::string ipc_path);

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

class ZmqPoller
{
public:
    ZmqPoller(int poll_timeout) : poll_timeout_(poll_timeout), running_(false) {};
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

class ZeroMqSubscriber
{
public:
    ZeroMqSubscriber(std::string &endpoint, std::string topic, ZmqPoller *poller)
        : endpoint_(endpoint), topic_(topic), poller_(poller)
    {
        context_ = new zmq::context_t(1);
        operation_ = new zmq::socket_t(*context_, zmq::socket_type::sub);
        operation_->connect(endpoint);
        operation_->set(zmq::sockopt::subscribe, topic_);
        std::cout << "[ZeroMqSubscriber] connect to " << endpoint.c_str() << std::endl;
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
    void setMessageHandler(MessageHandler handler, void *handler_param);

    void start();
    void stop();

private:
    std::string endpoint_;

    std::string topic_;
    zmq::socket_t *operation_ = nullptr;
    zmq::context_t *context_ = nullptr;
    ZmqPoller *poller_;
};
