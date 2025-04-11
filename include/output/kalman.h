#include "framework/module.h"
#include <iomanip>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

class Kalman
{
public:
    Kalman(const std::string &name,
           double time_slice,
           std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue,
           std::shared_ptr<std::mutex> inputLock,
           float sigma_a = 0.1f,
           size_t maxQueueLength = 0);

    ~Kalman();

    void setInputLock(std::shared_ptr<std::mutex> lock);
    void setOutputLock(std::shared_ptr<std::mutex> lock);
    void setInputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue);
    void setOutputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> outputQueue);
    std::shared_ptr<std::mutex> getOutputLock();
    std::shared_ptr<TimePriorityQueue<OutPackage>> getOutputQueue();

    void run();
    void stop();
    void join();

    static OutPackage kalman(std::vector<OutPackage> &pkgs, float sigma_a);

private:
    void process();

protected:
    std::string name;
    double timeSlice;
    std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue;
    std::shared_ptr<std::mutex> inputLock;
    std::shared_ptr<TimePriorityQueue<OutPackage>> outputQueue;
    std::shared_ptr<std::mutex> outputLock;
    float sigma_a;
    size_t maxQueueLength;
    std::atomic<bool> isRunning;
    std::thread thread_;
};
