#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include <vector>
#include <random>
#include <chrono>
#include "utils/zero_mq_client.h"
#include "utils/threadsafe_queue.h"
#include "utils/logger.h"
#include "framework/package_land_station.h"

// 消息处理回调函数
void messageHandler(void *param, zmq::message_t &message, const std::string &topic)
{
    threadsafe_queue<std::vector<uint8_t>> *message_queue = reinterpret_cast<threadsafe_queue<std::vector<uint8_t>> *>(param);
    if (message.size() != topic.size())
    {
        message_queue->push(std::vector<uint8_t>(static_cast<uint8_t *>(message.data()), static_cast<uint8_t *>(message.data()) + message.size()));
    }
}
int main()
{
    std::string ipc_path0 = "/tmp/zmq-test"; 
    std::string ipc_path1 = "/tmp/zmq-test1"; 

    std::string topic = "land_stream";
    std::string endpoint0 = "ipc://" + ipc_path0;
    std::string endpoint1 = "ipc://" + ipc_path1;

    std::vector<std::string> endpoints = {endpoint0, endpoint1};
    ZeroMqSubscriber subscriber(endpoints, topic, 10);
    threadsafe_queue<std::vector<uint8_t>> *message_queue = new threadsafe_queue<std::vector<uint8_t>>();
    subscriber.addMessageHandler(messageHandler, reinterpret_cast<void *>(message_queue));
    subscriber.start();

    while(true)
    {
        std::vector<uint8_t> m;
        message_queue->wait_and_pop(m);
        std::cout << "主线程收到消息大小: " <<m.size() << std::endl;

        auto inputdata = std::make_shared<DataPackageLandStation>();
        // 解析数据
        inputdata->parse_stream_location((uint8_t *)m.data(), m.size());
        
        // 解析数据
        uint64_t timestamp = inputdata->get_timestamp();
        std::cout << "时间戳: " << timestamp << std::endl;
        uint8_t uav_id = inputdata->get_uav_id();
        uint8_t camera_type = inputdata->get_camera_type();
        std::cout << "无人机ID: " << (int)uav_id << std::endl;
        std::cout << "相机类型: " << (int)camera_type << std::endl;
        float laser_distance = inputdata->get_laser_distance();
        std::cout << "激光距离: " << laser_distance << std::endl;
        uint8_t obj_num = inputdata->get_obj_num();
        std::cout << "目标数量: " << (int)obj_num << std::endl;
        std::vector<ObjectInfo> object_info = inputdata->get_object_info();
        for (const auto &obj : object_info)
        {
            std::cout << "跟踪器ID: " << (int)obj.tracker_id << std::endl;
            std::cout << "矩形框: (" << obj.rect.x << ", " << obj.rect.y << ", " << obj.rect.w << ", " << obj.rect.h << ")" << std::endl;
            std::cout << "标签: " << (int)obj.label << std::endl;
        }
        // 获取单应矩阵
        Homography homography = inputdata->get_homography();
        std::cout << "单应矩阵: ";
        for (int i = 0; i < 9; i++)
        {
            std::cout << homography.h[i] << " ";
        }
        std::cout << std::endl;
        // 获取相机矩阵
        CameraMatrix camera_matrix = inputdata->get_camera_matrix();
        std::cout << "相机矩阵: ";
        std::cout<<camera_matrix.member.x << " "
                 << camera_matrix.member.y << " "
                 << camera_matrix.member.z << " "
                 << camera_matrix.member.yaw << " "
                 << camera_matrix.member.pitch << " "
                 << camera_matrix.member.roll;
        std::cout << std::endl;
        // 获取RGB图像

        cv::Mat rgb = inputdata->get_rgb();
        cv::imwrite("/home/orin/ObjectLocation_cpp/data/test_result/r_" + std::to_string(inputdata->get_timestamp()) + ".jpg", rgb);
    }
    subscriber.stop();
}