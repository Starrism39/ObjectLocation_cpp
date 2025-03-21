#include <opencv2/imgcodecs.hpp>
#include "modules/fusion.h"


Fusion::Fusion(const std::string& name, 
            double time_slice, 
            std::shared_ptr<TimePriorityQueue<Package>> inputQueue,
            std::shared_ptr<std::mutex> inputLock,
            size_t maxQueueLength):
    name(name), 
    timeSlice(time_slice),
    maxQueueLength(maxQueueLength),
    inputQueue(std::move(inputQueue)),
    inputLock(std::move(inputLock))
{
    std::cout << "Building " << name << std::endl;
    auto OutputLock = std::make_shared<std::mutex>();
    auto OutputQueue = std::make_shared<TimePriorityQueue<OutPackage>>();
    setOutputLock(OutputLock);
    setOutputQueue(OutputQueue);
}

Fusion::~Fusion() {
    stop();
}

void Fusion::setInputLock(std::shared_ptr<std::mutex> lock){this->inputLock = lock;}

void Fusion::setOutputLock(std::shared_ptr<std::mutex> lock) {this->outputLock = lock;}

void Fusion::setInputQueue(std::shared_ptr<TimePriorityQueue<Package>> inputQueue){this->inputQueue = inputQueue;}

void Fusion::setOutputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> outputQueue) {
    this->outputQueue = outputQueue;
    if (maxQueueLength > 0 && outputQueue) {
        outputQueue->setMaxCount(maxQueueLength);
    }
}

std::shared_ptr<std::mutex> Fusion::getOutputLock(){return this->outputLock;}

std::shared_ptr<TimePriorityQueue<OutPackage>> Fusion::getOutputQueue(){return this->outputQueue;}

cv::Mat Fusion::cutTarget(cv::Mat img, std::vector<int> Bbox) {
    if (Bbox.size() != 4) {
        return cv::Mat();
    }

    int x = Bbox[0];
    int y = Bbox[1];
    int w = Bbox[2];
    int h = Bbox[3];

    if (w < 0) {
        x += w;
        w = -w;
    }
    if (h < 0) {
        y += h;
        h = -h;
    }

    cv::Rect roi(x, y, w, h);
    cv::Rect valid_roi = roi & cv::Rect(0, 0, img.cols, img.rows);

    // 检查交集区域是否有效
    if (valid_roi.width <= 0 || valid_roi.height <= 0) {
        return cv::Mat(); // 返回空矩阵
    }

    // 裁剪并返回目标图像
    return img(valid_roi).clone();
}

void Fusion::process(){
    while(isRunning){
        // std::cout << "running fusion" << std::endl;
        inputLock->lock();
        if(inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1){
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        std::vector<Package> packages = inputQueue->getTimeSlice(timeSlice);
        inputLock->unlock();

        if(packages.empty()) continue;

        OutPackage outpkg = fusion(packages);

        // 打印处理结果
        std::cout << std::string(3, '\n');
        std::cout << "==================== fusion ====================" << std::endl;
        std::cout << "fusion处理后的一个OutPackage有 " << outpkg.objs.size() << " 个目标" << std::endl;

        // 等待输出队列有空间
        while (outputQueue->isFull())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
            
        outputLock->lock();
        // 打印每个包的信息
        printOutPackage(outpkg);
        outputQueue->push(outpkg);
        outputLock->unlock();
    }

}

OutPackage Fusion::fusion(std::vector<Package>& pkgs) {
    OutPackage outpkg;
    if (pkgs.empty()) return outpkg;

    outpkg.time = pkgs[0].time;

    // 按global_id分组
    std::map<int, std::vector<Package>> id_groups;
    for (auto& pkg : pkgs) {
        id_groups[pkg.global_id].push_back(pkg);
    }

    // 构建Object
    for (auto& [gid, group] : id_groups) {
        Object obj;
        obj.global_id = gid;
        
        // 所有相同global_id的包使用第一个location
        if (!group.empty()) {
            obj.location = group[0].location;
        }

        // 4. 处理图像数据
        for (auto& pkg : group) {
            // 从本地读取原始图像（示例路径）
            std::string img_path = "/home/xjy/code/location_Map/test_1/data/received_image_23307y1m1d0h23m17s874ms.jpg";
            cv::Mat raw_img = cv::imread(img_path, cv::IMREAD_COLOR);
            
            if (!raw_img.empty()) {
                // 裁剪目标区域
                cv::Mat target_img = cutTarget(raw_img, pkg.Bbox);
                
                // 构建uav_id到图像的映射
                std::map<int, cv::Mat> img_map;
                try {
                    int uav_id = std::stoi(pkg.uav_id);
                    img_map[uav_id] = target_img;
                    obj.uav_img.push_back(img_map);
                } catch (...) {
                    std::cerr << "Invalid UAV ID: " << pkg.uav_id << std::endl;
                }
            }
        }

        outpkg.objs.push_back(obj);
    }

    return outpkg;
}


void Fusion::run(){
    isRunning = true;
    thread_ = std::thread(&Fusion::process, this);
}

void Fusion::stop() {
    if (isRunning) {
        isRunning = false;
        join();
    }
}

void Fusion::join(){
    if (thread_.joinable()) {
        thread_.join();
    }
}
