#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>
#include <map>
#include <unordered_set>
#include "output/kalman.h"

Kalman::Kalman(const std::string &name,
               double time_slice,
               std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue,
               std::shared_ptr<std::mutex> inputLock,
               float sigma_a,
               size_t maxQueueLength) : name(name),
                                        timeSlice(time_slice),
                                        inputQueue(std::move(inputQueue)), // 按照类中声明顺序调整
                                        inputLock(std::move(inputLock)),   // 在sigma_a和maxQueueLength之前
                                        sigma_a(sigma_a),
                                        maxQueueLength(maxQueueLength)
{
    std::cout << "\nBuilding " << name << std::endl;
    auto OutputLock = std::make_shared<std::mutex>();
    auto OutputQueue = std::make_shared<TimePriorityQueue<OutPackage>>();
    setOutputLock(OutputLock);
    setOutputQueue(OutputQueue);
}

Kalman::~Kalman()
{
    stop();
}

void Kalman::setInputLock(std::shared_ptr<std::mutex> lock) { this->inputLock = lock; }

void Kalman::setOutputLock(std::shared_ptr<std::mutex> lock) { this->outputLock = lock; }

void Kalman::setInputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> inputQueue) { this->inputQueue = inputQueue; }

void Kalman::setOutputQueue(std::shared_ptr<TimePriorityQueue<OutPackage>> outputQueue)
{
    this->outputQueue = outputQueue;
    if (maxQueueLength > 0 && outputQueue)
    {
        outputQueue->setMaxCount(maxQueueLength);
    }
}

std::shared_ptr<std::mutex> Kalman::getOutputLock() { return this->outputLock; }

std::shared_ptr<TimePriorityQueue<OutPackage>> Kalman::getOutputQueue() { return this->outputQueue; }

void Kalman::process()
{
    while (isRunning)
    {
        // std::cout << "running kalman" << std::endl;
        inputLock->lock();
        if (inputQueue->isEmpty() || inputQueue->deltaTime() < timeSlice + 1)
        {
            inputLock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        std::vector<OutPackage> packages = inputQueue->getSlidingWindow(timeSlice);
        inputLock->unlock();

        if (packages.empty())
            continue;

        OutPackage outpkg = kalman(packages, sigma_a);

        // // 打印处理结果
        // std::cout << std::string(3, '\n');
        // std::cout << "==================== kalman ====================" << std::endl;
        // std::cout << "kalman处理后的一个OutPackage有 " << outpkg.objs.size() << " 个目标" << std::endl;

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

OutPackage Kalman::kalman(std::vector<OutPackage> &pkgs, float sigma_a)
{
    OutPackage result;
    if (pkgs.empty())
    {
        return result;
    }

    // 反转以按时间升序处理（从旧到新）
    std::reverse(pkgs.begin(), pkgs.end());

    // 获取最新数据包（反转后的最后一个元素，即原 pkgs[0]）中的所有 global_id
    const auto &latest_pkg = pkgs.back();
    std::unordered_set<int> valid_ids;
    for (const auto &obj : latest_pkg.objs)
    {
        valid_ids.insert(obj.global_id);
    }

    result.time = latest_pkg.time; // 使用最新包的时间
    result.time_slice = latest_pkg.time_slice;
    result.uav_pose = latest_pkg.uav_pose;

    // 维护每个目标的卡尔曼滤波器和上次处理时间
    std::map<int, cv::KalmanFilter> kf_map;
    std::map<int, time_t> last_time_map;

    const int state_dim = 6; // x, y, z, vx, vy, vz
    const int meas_dim = 3;  // x, y, z

    for (auto &pkg : pkgs)
    {
        for (auto &obj : pkg.objs)
        {
            int id = obj.global_id;

            // 仅处理 valid_ids 中的目标
            if (valid_ids.find(id) == valid_ids.end())
            {
                continue;
            }

            if (kf_map.find(id) == kf_map.end())
            {
                // 初始化卡尔曼滤波器
                cv::KalmanFilter kf(state_dim, meas_dim, 0);

                // 转移矩阵初始为单位矩阵，后续动态调整 dt
                kf.transitionMatrix = cv::Mat::eye(state_dim, state_dim, CV_32F);

                // 测量矩阵
                kf.measurementMatrix = cv::Mat::zeros(meas_dim, state_dim, CV_32F);
                kf.measurementMatrix.at<float>(0, 0) = 1.0f;
                kf.measurementMatrix.at<float>(1, 1) = 1.0f;
                kf.measurementMatrix.at<float>(2, 2) = 1.0f;

                // 过程噪声协方差（初始值，后续动态调整）
                cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-5));
                // 测量噪声协方差
                cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
                // 初始状态协方差
                cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

                // 初始化状态：位置为观测值，速度为0
                cv::Mat state = cv::Mat::zeros(state_dim, 1, CV_32F);
                for (int i = 0; i < 3 && i < obj.location.size(); ++i)
                {
                    state.at<float>(i) = static_cast<float>(obj.location[i]);
                }
                kf.statePost = state.clone();

                kf_map[id] = kf;
                last_time_map[id] = pkg.time;
            }
            else
            {
                cv::KalmanFilter &kf = kf_map[id];
                time_t last_time = last_time_map[id];
                double dt = difftime(pkg.time, last_time);

                if (dt > 0)
                {
                    // 更新转移矩阵中的 dt
                    for (int i = 0; i < 3; ++i)
                    {
                        kf.transitionMatrix.at<float>(i, i + 3) = static_cast<float>(dt);
                    }

                    // 预测
                    kf.predict();

                    // 计算过程噪声协方差 Q
                    float dt2 = dt * dt;
                    float dt3 = dt2 * dt;
                    float dt4 = dt3 * dt;

                    cv::Mat Q = cv::Mat::zeros(state_dim, state_dim, CV_32F);
                    for (int i = 0; i < 3; ++i)
                    {
                        Q.at<float>(i, i) = 0.25f * dt4 * sigma_a * sigma_a;
                        Q.at<float>(i, i + 3) = 0.5f * dt3 * sigma_a * sigma_a;
                        Q.at<float>(i + 3, i) = 0.5f * dt3 * sigma_a * sigma_a;
                        Q.at<float>(i + 3, i + 3) = dt2 * sigma_a * sigma_a;
                    }

                    kf.processNoiseCov = Q;
                }

                // 准备测量值
                cv::Mat measurement = cv::Mat::zeros(meas_dim, 1, CV_32F);
                for (int i = 0; i < 3 && i < obj.location.size(); ++i)
                {
                    measurement.at<float>(i) = static_cast<float>(obj.location[i]);
                }

                // 更新
                kf.correct(measurement);
                last_time_map[id] = pkg.time;
            }
        }
    }

    // 收集结果：仅包含 latest_pkg 中的 global_id
    for (const auto &obj : latest_pkg.objs)
    {
        int id = obj.global_id;
        auto it = kf_map.find(id);
        if (it == kf_map.end())
        {
            continue; // 该 ID 未被处理（理论上不会发生）
        }

        const cv::KalmanFilter &kf = it->second;
        Object filtered_obj;

        // 复制全局 ID
        filtered_obj.global_id = id;

        // 位置为滤波后的状态
        cv::Mat state = kf.statePost;
        for (int i = 0; i < 3; ++i)
        {
            filtered_obj.location.push_back(static_cast<double>(state.at<float>(i)));
        }

        // 从最新包中复制其他字段（如图像）
        filtered_obj.uav_img = obj.uav_img;

        result.objs.push_back(filtered_obj);
    }

    return result;
}

void Kalman::run()
{
    isRunning = true;
    thread_ = std::thread(&Kalman::process, this);
}

void Kalman::stop()
{
    if (isRunning)
    {
        isRunning = false;
        join();
    }
}

void Kalman::join()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}
