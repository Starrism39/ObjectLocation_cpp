#include <opencv2/imgcodecs.hpp>
#include "modules/fusion.h"

Fusion::Fusion(const std::string &name,
               double time_slice,
               std::shared_ptr<TimePriorityQueue<Package>> inputQueue,
               std::shared_ptr<std::mutex> inputLock,
               size_t maxQueueLength) : name(name),
                                        timeSlice(time_slice),
                                        inputQueue(std::move(inputQueue)),
                                        inputLock(std::move(inputLock)),
                                        maxQueueLength(maxQueueLength)
{
    std::cout << "Building " << name << std::endl;
    auto OutputLock = std::make_shared<std::mutex>();
    auto OutputQueue = std::make_shared<TimePriorityQueue<OutPackage>>();
    setOutputLock(OutputLock);
    setOutputQueue(OutputQueue);
}

Fusion::~Fusion()
{
    stop();
}

void Fusion::setInputLock(std::shared_ptr<std::mutex> lock) { this->inputLock = lock; }

void Fusion::setOutputLock(std::shared_ptr<std::mutex> lock) { this->outputLock = lock; }

void Fusion::setInputQueue(std::shared_ptr<TimePriorityQueue<Package>> inputQueue) { this->inputQueue = inputQueue; }

void Fusion::setOutputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> outputQueue)
{
    this->outputQueue = outputQueue;
    if (maxQueueLength > 0 && outputQueue)
    {
        outputQueue->setMaxCount(maxQueueLength);
    }
}

std::shared_ptr<std::mutex> Fusion::getOutputLock() { return this->outputLock; }

std::shared_ptr<TimePriorityQueue<OutPackage>> Fusion::getOutputQueue() { return this->outputQueue; }

cv::Mat Fusion::cutTarget(cv::Mat img, std::vector<int> Bbox)
{
    if (Bbox.size() != 4)
    {
        return cv::Mat();
    }

    int x = Bbox[0];
    int y = Bbox[1];
    int w = Bbox[2];
    int h = Bbox[3];

    if (w < 0)
    {
        x += w;
        w = -w;
    }
    if (h < 0)
    {
        y += h;
        h = -h;
    }

    cv::Rect roi(x, y, w, h);
    cv::Rect valid_roi = roi & cv::Rect(0, 0, img.cols, img.rows);

    // 检查交集区域是否有效
    if (valid_roi.width <= 0 || valid_roi.height <= 0)
    {
        return cv::Mat(); // 返回空矩阵
    }

    // 裁剪并返回目标图像
    return img(valid_roi).clone();
}

OutPackage Fusion::fusion(std::vector<Package> &pkgs, double timeSlice)
{
    OutPackage outpkg;
    if (pkgs.empty())
        return outpkg;

    outpkg.time = pkgs[0].time;

    outpkg.time_slice = timeSlice;

    std::map<uint8_t, std::vector<double>> uav_pose_map;
    for (const auto &pkg : pkgs) {
        if (!pkg.uav_id.empty()) {
            try {
                int uav_id = std::stoi(pkg.uav_id);
                uav_pose_map[uav_id] = pkg.camera_pose;
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid UAV ID format: " << pkg.uav_id << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "UAV ID out of range: " << pkg.uav_id << std::endl;
            }
        }
    }
    outpkg.uav_pose = uav_pose_map;

    // 按global_id分组
    std::map<int, std::vector<Package>> id_groups;
    for (auto &pkg : pkgs)
    {
        id_groups[pkg.global_id].push_back(pkg);
    }

    // 构建Object
    for (auto &[gid, group] : id_groups)
    {
        Object obj;
        obj.global_id = gid;

        // 所有相同global_id的包使用第一个location
        if (!group.empty())
        {
            obj.location = group[0].location;
        }

        for (auto &pkg : group) {
            try {
                // 实际应从pkg获取图像路径
                std::string img_path = "/home/orin/ObjectLocation_cpp/data/received_image_23307y1m1d0h23m17s874ms.jpg";
                cv::Mat raw_img = cv::imread(img_path, cv::IMREAD_COLOR);
                
                if (!raw_img.empty()) {
                    cv::Mat target_img = cutTarget(raw_img, pkg.Bbox);
                    int uav_id = std::stoi(pkg.uav_id);
                    // 直接插入到uav_img map
                    obj.uav_img[uav_id] = target_img;
                }
            } catch (const std::exception& e) {
                std::cerr << "Image processing error: " << e.what() << std::endl;
            }
        }

        outpkg.objs.push_back(obj);
    }

    return outpkg;
}

void Fusion::process()
{
    while (isRunning)
    {
        // std::cout << "running fusion" << std::endl;
        inputLock->lock();
        if (inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1)
        {
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        std::vector<Package> packages = inputQueue->getTimeSlice(timeSlice);
        inputLock->unlock();

        if (packages.empty())
            continue;

        OutPackage outpkg = fusion(packages, this->timeSlice);

        // // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== fusion ====================" << std::endl;
        // std::cout << "fusion处理后的一个OutPackage有 " << outpkg.objs.size() << " 个目标" << std::endl;

        // 等待输出队列有空间
        while (outputQueue->isFull())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        outputLock->lock();
        // // 打印每个包的信息
        // printOutPackage(outpkg);
        outputQueue->push(outpkg);
        outputLock->unlock();
    }
}

void Fusion::run()
{
    isRunning = true;
    thread_ = std::thread(&Fusion::process, this);
}

void Fusion::stop()
{
    if (isRunning)
    {
        isRunning = false;
        join();
    }
}

void Fusion::join()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}
