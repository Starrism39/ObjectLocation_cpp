#include"framework/module.h"
#include <iomanip>
#include <map>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>


class Fusion{
public:
    Fusion(const std::string& name, 
        double time_slice, 
        std::shared_ptr<TimePriorityQueue<Package>> inputQueue,
        std::shared_ptr<std::mutex> inputLock,
        size_t maxQueueLength = 0); 

    ~Fusion();

    void setInputLock(std::shared_ptr<std::mutex> lock);
    void setOutputLock(std::shared_ptr<std::mutex> lock);
    void setInputQueue(std::shared_ptr<TimePriorityQueue<Package>> inputQueue);
    void setOutputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> outputQueue);
    std::shared_ptr<std::mutex> getOutputLock();
    std::shared_ptr<TimePriorityQueue<OutPackage>> getOutputQueue();

    void run();
    void stop();
    void join();

    static cv::Mat cutTarget(cv::Mat img, std::vector<int> Bbox);
    static OutPackage fusion(std::vector<Package>& pkgs);

protected:
    double timeSlice;

private:
    void process();
    

protected:
    std::string name;
    std::shared_ptr<TimePriorityQueue<Package>> inputQueue;
    std::shared_ptr<std::mutex> inputLock;
    std::shared_ptr<TimePriorityQueue<OutPackage>> outputQueue;
    std::shared_ptr<std::mutex> outputLock;
    size_t maxQueueLength;
    std::atomic<bool> isRunning;
    std::thread thread_;
};

