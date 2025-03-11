#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "yaml-cpp/yaml.h"

#include "framework/pipeline.h"

// TODO：输入、输出
#include "modules/converter.h"
#include "modules/time_filter.h"
#include "modules/esti_position.h"
#include "modules/spatial_filter.h"

// 创建测试用的ObjectInfo
ObjectInfo CreateTestObject(uint8_t uid, uint16_t tracker_id, uint8_t label,
                            uint16_t norm_x = 0.5, uint16_t norm_y = 0.5, uint16_t norm_w = 0.2, uint16_t norm_h = 0.3)
{
    // 添加一点随机性，但保持变化较小
    double random_offset_x = (rand() % 100 - 50) / 50.0; // -1到1的随机偏移
    double random_offset_y = (rand() % 100 - 50) / 50.0;

    double adjusted_norm_x = norm_x + random_offset_x;
    double adjusted_norm_y = norm_y + random_offset_y;

    // 确保值在合理范围内
    adjusted_norm_x = std::max(0.0, std::min(1.0, adjusted_norm_x));
    adjusted_norm_y = std::max(0.0, std::min(1.0, adjusted_norm_y));

    ObjectInfo obj;
    obj.uid = uid;
    obj.tracker_id = tracker_id;
    obj.rect = {
        static_cast<uint16_t>(adjusted_norm_x * 1920), // x
        static_cast<uint16_t>(adjusted_norm_y * 1080), // y
        static_cast<uint16_t>(norm_w * 1920),          // w
        static_cast<uint16_t>(norm_h * 1080)           // h
    };
    obj.prob = 0.95 + (rand() % 5) / 100.0; // 概率值小幅波动
    obj.label = label;

    double cx = adjusted_norm_x + norm_w / 2;
    double cy = adjusted_norm_y + norm_h / 2;

    // 相机内参计算（使用camera_K）
    double world_x = cx * 0.01 + 0.001;
    double world_y = cy * -0.02 + 0.002;

    // 坐标转换，添加微小随机偏移
    double pos_noise = (rand() % 20 - 10) / 100.0;        // -0.1到0.1的随机偏移
    obj.wgs84[0] = 500 + world_x * 0.1 + pos_noise;       // X
    obj.wgs84[1] = 600 + world_y * 0.1 + pos_noise;       // Y
    obj.wgs84[2] = 200 - 50.0 + (rand() % 10 - 5) / 10.0; // Z (高度微调)
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
        500.0, 600.0, 200.0, // x, y, z
        1.57, 0.0, 0.0       // yaw, pitch, roll
    };
    data_pkg->set_camera_info(uav_id, camera_type, camera_matrix);

    // 添加目标信息
    data_pkg->set_obj_num(objects.size());
    for (const auto &obj : objects)
    {
        data_pkg->push_object_info(obj);
    }

    return data_pkg;
}

// 工厂函数，根据配置创建模块实例
Module *createModule(const std::string &name, const YAML::Node &args)
{

    if (name == "TimeFilter")
    {
        return new TimeFilter(
            args["time_slice"].as<double>(),
            args["max_queue_length"].IsDefined() ? args["max_queue_length"].as<int>() : 0);
    }

    else if (name == "EstiPosition")
    {
        if (args["is_multi_map"].as<bool>())
        {
            // 多地图版本
            return new EstiPosition(
                args["is_multi_map"].as<bool>(),
                args["map_path"].as<std::string>(),
                args["default_height"].as<double>(),
                args["order"].as<std::string>(),
                args["enable_reloaction"].as<bool>(),
                args["max_queue_length"].IsDefined() ? args["max_queue_length"].as<int>() : 0,
                args["multi_num"].as<int>(),
                [&args]()
                {
                    std::vector<double> scale;
                    for (const auto &val : args["downSampled_scale"])
                    {
                        scale.push_back(val.as<double>());
                    }
                    return scale;
                }(),
                [&args]()
                {
                    std::vector<std::vector<double>> scale;
                    for (const auto &outer : args["split_scale"])
                    {
                        std::vector<double> inner;
                        for (const auto &val : outer)
                        {
                            inner.push_back(val.as<double>());
                        }
                        scale.push_back(inner);
                    }
                    return scale;
                }(),
                args["block_num"].as<int>(),
                args["overlap_x_scale"].as<double>(),
                args["overlap_y_scale"].as<double>());
        }
        else
        {
            // 单地图版本
            return new EstiPosition(
                args["is_multi_map"].as<bool>(),
                args["map_path"].as<std::string>(),
                args["default_height"].as<double>(),
                args["order"].as<std::string>(),
                args["enable_reloaction"].as<bool>(),
                args["max_queue_length"].IsDefined() ? args["max_queue_length"].as<int>() : 0);
        }
    }

    else if (name == "SpatialFilter")
    {
        return new SpatialFilter(
            args["time_slice"].as<double>(),
            args["distance_threshold"].as<double>(),
            args["max_map"].as<int>(),
            args["max_queue_length"].IsDefined() ? args["max_queue_length"].as<int>() : 0);
    }

    throw std::runtime_error("Unknown module type: " + name);
}

int main(int argc, char *argv[])
{
    // 设置默认配置文件路径
    std::string config_path = "config.yaml";

    // 如果提供了命令行参数，使用指定的配置文件
    if (argc > 1)
    {
        config_path = argv[1];
    }

    try
    {

        // 读取YAML配置文件
        YAML::Node config = YAML::LoadFile(config_path);

        // 创建数据转换线程
        auto ConInputQueue = std::make_shared<std::vector<std::shared_ptr<DataPackage>>>();
        auto ConOutputQueue = std::make_shared<TimePriorityQueue>();
        auto ConInputLock = std::make_shared<std::mutex>();
        auto ConOutputLock = std::make_shared<std::mutex>();

        std::string module_name = config["input"]["stage1"]["name"].as<std::string>();
        YAML::Node args = config["input"]["stage1"]["args"];
        ConOutputQueue->setMaxCount(args["max_queue_length"].IsDefined() ? args["max_queue_length"].as<int>() : 0);

        PackageConverter converter(module_name, ConInputQueue, ConOutputQueue, ConInputLock, ConOutputLock);

        // 创建处理管道（处理）
        // 存储所有管道阶段的模块
        std::vector<std::vector<Module *>> pipelines;

        // 遍历pipeline节点
        for (const auto &stage : config["pipeline"])
        {
            std::vector<Module *> stage_modules;

            // 获取模块名称和参数
            module_name = stage.second["name"].as<std::string>();
            YAML::Node args = stage.second["args"];
            int parallel = stage.second["parallel"].as<int>();

            // 创建指定数量的并行模块
            for (int i = 0; i < parallel; ++i)
            {
                Module *module = createModule(module_name, args);
                stage_modules.push_back(module);
            }

            pipelines.push_back(stage_modules);
        }

        // 启动转换器线程
        converter.run();
        // 创建并运行管道
        Pipeline pipeline(pipelines, ConOutputQueue, ConOutputLock);

        // ======================
        // 测试流水线
        // ======================

        // 生成测试数据
        srand(time(nullptr));

        // 创建测试数据包
        std::vector<std::shared_ptr<DataPackage>> test_packages;

        // 场景1：电视相机，多个目标
        std::vector<ObjectInfo> objects1 = {
            CreateTestObject(0, 1, 1),
            CreateTestObject(0, 2, 2),
            CreateTestObject(0, 3, 1)};
        test_packages.push_back(CreateTestDatapackage(1634567890, 1, 0, objects1));

        // 场景2：红外相机，多个目标
        std::vector<ObjectInfo> objects2 = {
            CreateTestObject(0, 1, 3),
            CreateTestObject(0, 2, 3)};
        test_packages.push_back(CreateTestDatapackage(1634567892, 2, 1, objects2));

        // 场景3：微光相机，多个目标
        std::vector<ObjectInfo> objects3 = {
            CreateTestObject(0, 1, 1),
            CreateTestObject(0, 2, 4),
            CreateTestObject(0, 3, 3),
            CreateTestObject(0, 4, 2)};
        test_packages.push_back(CreateTestDatapackage(1634567894, 3, 2, objects3));

        // 场景3：微光相机，多个目标
        std::vector<ObjectInfo> objects4 = {
            CreateTestObject(0, 1, 1),
            CreateTestObject(0, 2, 4),
            CreateTestObject(0, 3, 3)};
        test_packages.push_back(CreateTestDatapackage(1634567896, 3, 2, objects4));

        // 场景4：场景2：红外相机，多个目标
        std::vector<ObjectInfo> objects5 = {
            CreateTestObject(0, 1, 1),
            CreateTestObject(0, 2, 4),
            CreateTestObject(0, 3, 3)};
        test_packages.push_back(CreateTestDatapackage(1634567898, 2, 1, objects5));

        // 输入数据到流水线
        std::cout << "==================== 开始测试流水线 ====================" << std::endl;
        for (const auto &data_pkg : test_packages)
        {
            // 添加数据到输入队列
            {
                std::lock_guard<std::mutex> guard(*ConInputLock);
                ConInputQueue->push_back(data_pkg);
                std::cout << "添加数据包到输入队列，时间戳: " << data_pkg->get_timestamp()
                          << "，相机类型: " << (data_pkg->get_camera_type() == 0 ? "电视" : data_pkg->get_camera_type() == 1 ? "红外"
                                                                                                                             : "微光")
                          << "，目标数量: " << static_cast<int>(data_pkg->get_obj_num()) << std::endl;
            }
        }
        std::cout << "添加后队列大小: " << ConInputQueue->size() << std::endl;


        converter.join();
        pipeline.join();
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}