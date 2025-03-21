#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <memory>
#include "yaml-cpp/yaml.h"

#include"framework/pipeline.h"


#include"modules/converter.h"

// 创建测试用的ObjectInfo
ObjectInfo CreateTestObject(uint8_t uid, uint16_t tracker_id,
                          uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                          uint8_t label)
{
    ObjectInfo obj;
    obj.uid = uid;
    obj.tracker_id = tracker_id;
    obj.rect = {x, y, w, h};
    obj.prob = 0.95;
    obj.label = label;
    obj.wgs84[0] = 116.0 + (static_cast<double>(rand()) / RAND_MAX);
    obj.wgs84[1] = 39.0 + (static_cast<double>(rand()) / RAND_MAX);
    obj.wgs84[2] = 100.0 + (rand() % 100);
    return obj;
}

std::shared_ptr<DataPackage> CreateTestDatapackage(uint64_t timestamp, uint8_t uav_id,
    uint8_t camera_type,
    const std::vector<ObjectInfo> &objects)
{
// 使用make_shared替代make_unique
auto data_pkg = std::make_shared<DataPackage>();

// 设置时间戳
data_pkg->set_timestamp(timestamp);

// 设置相机信息
double camera_matrix[6] = {
100.0 + static_cast<double>(rand() % 100),
200.0 + static_cast<double>(rand() % 100),
300.0 + static_cast<double>(rand() % 100),
static_cast<double>(rand() % 360),
static_cast<double>(rand() % 90),
static_cast<double>(rand() % 360)};
data_pkg->set_camera_info(uav_id, camera_type, camera_matrix);

// 添加目标信息
data_pkg->set_obj_num(objects.size());
for (const auto &obj : objects)
{
data_pkg->push_object_info(obj);
}

return data_pkg;
}

int main() {
    try {
        srand(time(nullptr));

        // 创建共享队列和互斥锁
        auto inputQueue = std::make_shared<std::vector<std::shared_ptr<DataPackage>>>();
        auto inputLock = std::make_shared<std::mutex>();
        

        
        // 创建PackageConverter实例
        PackageConverter converter("test_converter", inputQueue, inputLock, 1000);
        auto outputQueue = converter.getOutputQueue();
        auto outputLock = converter.getOutputLock();
        
        // 启动转换器线程
        converter.run();
        
        // 准备测试数据包
        std::vector<std::shared_ptr<DataPackage>> test_packages;

        // 场景1：电视相机，多个目标
        std::vector<ObjectInfo> objects1 = {
            CreateTestObject(1, 100, 100, 200, 50, 30, 1),
            CreateTestObject(2, 101, 300, 400, 40, 60, 2),
            CreateTestObject(3, 102, 500, 600, 35, 45, 1)
        };
        test_packages.push_back(CreateTestDatapackage(1634567890, 1, 0, objects1));

        // 场景2：红外相机，多个目标
        std::vector<ObjectInfo> objects2 = {
            CreateTestObject(4, 200, 150, 250, 45, 35, 3),
            CreateTestObject(5, 201, 350, 450, 55, 40, 2)
        };
        test_packages.push_back(CreateTestDatapackage(1634567891, 2, 1, objects2));

        // 场景3：微光相机，多个目标
        std::vector<ObjectInfo> objects3 = {
            CreateTestObject(6, 300, 200, 300, 40, 50, 1),
            CreateTestObject(7, 301, 400, 500, 30, 40, 4),
            CreateTestObject(8, 302, 600, 700, 45, 55, 3),
            CreateTestObject(9, 303, 800, 900, 35, 45, 2)
        };
        test_packages.push_back(CreateTestDatapackage(1634567892, 3, 2, objects3));

        // 方式1：测试静态转换方法
        std::cout << "\n===== 测试静态转换方法 =====" << std::endl;
        for (size_t i = 0; i < test_packages.size(); ++i) {
            std::cout << "\n========== 数据包 " << i + 1 << " ==========" << std::endl;
            const auto &data_pkg = test_packages[i];

            std::cout << "相机类型: " << (data_pkg->get_camera_type() == 0 ? "电视" : 
                                      data_pkg->get_camera_type() == 1 ? "红外" : "微光") << std::endl;
            std::cout << "目标数量: " << static_cast<int>(data_pkg->get_obj_num()) << std::endl;

            auto packages = PackageConverter::ConvertToPackages(data_pkg);
            
            for (size_t j = 0; j < packages.size(); ++j) {
                std::cout << "\n--- 目标 " << j + 1 << " ---" << std::endl;
                PrintPackage(packages[j]);
            }
        }

        // 方式2：测试线程处理
        std::cout << "\n===== 测试线程处理 =====" << std::endl;
        for (const auto& data_pkg : test_packages) {
            // 添加数据到输入队列
            {
                std::lock_guard<std::mutex> guard(*inputLock);
                inputQueue->push_back(data_pkg);
                std::cout << "添加数据包到输入队列，时间戳: " << data_pkg->get_timestamp() << std::endl;
            }
            
            // 等待一小段时间让转换器处理数据
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 等待处理完成
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 从输出队列中获取结果
        {
            std::lock_guard<std::mutex> guard(*outputLock);
            std::cout << "\n输出队列中的Package数量: " << outputQueue->size() << std::endl;
            
            while (!outputQueue->isEmpty()) {
                Package pkg = outputQueue->pop();
                std::cout << "\n从输出队列获取Package:" << std::endl;
                PrintPackage(pkg);
            }
        }

        // 停止转换器线程
        converter.stop();
        
        std::cout << "\n测试完成！" << std::endl;
    }
    catch (const std::exception &e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
