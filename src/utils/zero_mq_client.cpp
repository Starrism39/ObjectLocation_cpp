#include <iostream> // 新增头文件
#include "utils/zero_mq_client.h"

void ZeroMqPublisher::publish(uint8_t *data_buffer, size_t send_len)
{
    try
    {
        zmq::message_t topicMsg(topic_.size());
        memcpy(topicMsg.data(), topic_.data(), topic_.size());
        operation_->send(topicMsg, zmq::send_flags::sndmore);

        zmq::message_t message(send_len);
        memcpy(message.data(), data_buffer, send_len);
        operation_->send(message, zmq::send_flags::none);
    }
    catch (std::exception &e)
    {
        std::cerr << "ZeroMqPublisher Publish客户端 #" << endpoint_ << " 异常: " << e.what() << std::endl;
    }
}

void ZeroMqSubscriber::addMessageHandler(MessageHandler handler, void *handler_param)
{
    if (operation_ == nullptr)
    {
        std::cerr << "ZeroMqSubscriber 操作套接字为空，无法设置消息处理函数" << std::endl;
        return;
    }
    if (handler == nullptr)
    {
        std::cerr << "ZeroMqSubscriber 消息处理函数为空，无法设置" << std::endl;
        return;
    }
    this->addSocket(operation_, [handler](void *param, zmq::message_t &message, std::string &topic)
                    { handler(param, message, topic); }, handler_param, topic_);
}

void ZmqPoller::addSocket(zmq::socket_t *socket, MessageHandler handler, void *handler_param, std::string &topic)
{
    Socket s;
    s.socket = socket;
    s.handler = handler;
    s.handler_param = handler_param;
    s.topic = topic;
    sockets_.push_back(s);
    std::cout << "ZmqPoller 添加套接字到轮询器: " << topic << std::endl;
}

void ZmqPoller::start()
{
    if (running_)
    {
        std::cerr << "ZmqPoller 轮询器线程已经在运行" << std::endl;
        return;
    }
    if (sockets_.empty())
    {
        std::cerr << "ZmqPoller 没有可用的套接字" << std::endl;
        return;
    }
    running_ = true;
    poller_thread_ = std::thread(&ZmqPoller::run, this);
    std::cout << "ZmqPoller 轮询器线程已启动" << std::endl;
}
void ZmqPoller::stop()
{
    if (!running_)
    {
        std::cerr << "ZmqPoller 轮询器已经停止" << std::endl;
        return;
    }
    running_ = false;
    if (poller_thread_.joinable())
    {
        poller_thread_.join();
    }
    std::cout << "ZmqPoller 轮询器线程已停止" << std::endl;
}

void ZmqPoller::run()
{
    std::vector<zmq::pollitem_t> pollitems;

    // 为每个套接字创建轮询项
    for (const auto &s : sockets_)
    {
        zmq::pollitem_t item = {static_cast<void *>(*s.socket), 0, ZMQ_POLLIN, 0};
        pollitems.push_back(item);
    }

    // 循环处理消息
    while (running_)
    {
        try
        {
            // 轮询套接字
            zmq::poll(pollitems.data(), pollitems.size(), std::chrono::milliseconds(poll_timeout_));
            // 检查每个套接字是否有消息
            for (size_t i = 0; i < pollitems.size(); ++i)
            {
                if (pollitems[i].revents & ZMQ_POLLIN)
                {
                    // 接收消息
                    zmq::message_t message;
                    auto result = sockets_[i].socket->recv(message, zmq::recv_flags::none);
                    // 调用处理函数
                    if (result)
                    {
                        // 处理消息
                        sockets_[i].handler(sockets_[i].handler_param, message, sockets_[i].topic);
                    }
                    else
                    {
                        std::cerr << "ZmqPoller 接收消息失败" << std::endl;
                    }
                }
            }
        }
        catch (const zmq::error_t &e)
        {
            std::cerr << "ZmqPoller 轮询器异常: " << e.what() << std::endl;
        }
    }
}