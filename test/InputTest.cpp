#include "input/input.h"

int main()
{
    // 初始化输入模块
    std::vector<std::string> endpoints = {"ipc:///tmp/zmq-test", "ipc:///tmp/zmq-test1"};
    Input input("input", endpoints, "land_stream");

    // 设置输出队列
    auto outputQueue = input.getOutputQueue();

    // 启动处理
    input.run();

    // 主线程可消费输出队列
    while (true)
    {
        std::lock_guard<std::mutex> lock(*input.getOutputLock());
        if (!outputQueue->empty())
        {
            std::shared_ptr<DataPackage> pkg = outputQueue->back();
            outputQueue->pop_back();

            // 解析数据
            uint64_t timestamp = pkg->get_timestamp();
            std::cout << "时间戳: " << timestamp << std::endl;
            uint8_t uav_id = pkg->get_uav_id();
            uint8_t camera_type = pkg->get_camera_type();
            std::cout << "无人机ID: " << (int)uav_id << std::endl;
            std::cout << "相机类型: " << (int)camera_type << std::endl;
            float laser_distance = pkg->get_laser_distance();
            std::cout << "激光距离: " << laser_distance << std::endl;
            uint8_t obj_num = pkg->get_obj_num();
            std::cout << "目标数量: " << (int)obj_num << std::endl;
            std::vector<ObjectInfo> object_info = pkg->get_object_info();
            for (const auto &obj : object_info)
            {
                std::cout << "目标ID: " << (int)obj.uid << std::endl;
                std::cout << "跟踪器ID: " << (int)obj.tracker_id << std::endl;
                std::cout << "矩形框: (" << obj.rect.x << ", " << obj.rect.y << ", " << obj.rect.w << ", " << obj.rect.h << ")" << std::endl;
                std::cout << "置信度: " << obj.prob << std::endl;
                std::cout << "标签: " << (int)obj.label << std::endl;
                std::cout << "WGS84坐标: (" << obj.wgs84[0] << ", " << obj.wgs84[1] << ", " << obj.wgs84[2] << ")" << std::endl;
            }
            // 获取单应矩阵
            Homography homography = pkg->get_homography();
            std::cout << "单应矩阵: ";
            for (int i = 0; i < 9; i++)
            {
                std::cout << homography.h[i] << " ";
            }
            std::cout << std::endl;
            // 获取相机矩阵
            CameraMatrix camera_matrix = pkg->get_camera_matrix();
            std::cout << "相机矩阵: ";
            std::cout << camera_matrix.member.x << " "
                      << camera_matrix.member.y << " "
                      << camera_matrix.member.z << " "
                      << camera_matrix.member.yaw << " "
                      << camera_matrix.member.pitch << " "
                      << camera_matrix.member.roll;
            std::cout << std::endl;
            cv::Mat rgb = pkg->get_rgb();
            cv::imwrite("/home/orin/ObjectLocation_cpp/data/test_result/r_" + std::to_string(pkg->get_timestamp()) + ".jpg", rgb);
        }
    }

    input.stop();
    return 0;
}
