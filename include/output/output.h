#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <map>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "framework/module.h"
#include "utils/sendFrament.h"

class Output
{
public:
    Output(const std::string &name,
           const std::string &ip,
           int port,
           const std::string &interface,
           std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue,
           std::shared_ptr<std::mutex> inputLock);

    ~Output();

    void setInputLock(std::shared_ptr<std::mutex> lock);
    void setInputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue);

    void run();
    void stop();
    void join();

    static bool serializeOutPackage(const OutPackage& pkg, std::vector<uint8_t>& buffer);
    void output(const OutPackage& pkg);

private:
    void processObjectsInCircle(OutPackage& pkg, double center_x, double center_y, double radius);
    void processObjectsInRectangle(OutPackage& pkg, double min_x, double max_x, double min_y, double max_y);
    void process();

protected:
    std::string name;
    std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue;
    std::shared_ptr<std::mutex> inputLock;
    std::string ip;
    std::shared_ptr<UDPOperation> server;
    std::atomic<bool> isRunning;
    std::thread thread_;
};
