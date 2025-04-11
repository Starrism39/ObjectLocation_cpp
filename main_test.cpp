#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "yaml-cpp/yaml.h"

#include "framework/pipeline.h"


#include "input/converter.h"

#include "modules/time_filter.h"
#include "modules/esti_position.h"
#include "modules/spatial_filter.h"

#include "output/fusion.h"
#include "output/kalman.h"
#include "output/output.h"

// 创建测试用的ObjectInfo
ObjectInfo CreateTestObject(uint8_t uid, uint16_t tracker_id, uint8_t label,
                            float norm_x = 0.5f, float norm_y = 0.5f, float norm_w = 0.2f, float norm_h = 0.3f)
{
    // 添加较小的随机偏移（±0.1）
    float random_offset_x = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.2f;
    float random_offset_y = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.2f;

    // 调整坐标，确保在0-1范围内
    float adjusted_x = std::max(0.0f, std::min(1.0f, norm_x + random_offset_x));
    float adjusted_y = std::max(0.0f, std::min(1.0f, norm_y + random_offset_y));

    // 确保边界框不会超出图像边界
    adjusted_x = std::min(adjusted_x, 1.0f - norm_w);
    adjusted_y = std::min(adjusted_y, 1.0f - norm_h);

    ObjectInfo obj;
    obj.uid = uid;
    obj.tracker_id = tracker_id;
    obj.rect = {
        static_cast<uint16_t>(adjusted_x * 1920), // x
        static_cast<uint16_t>(adjusted_y * 1080), // y
        static_cast<uint16_t>(norm_w * 1920),     // w
        static_cast<uint16_t>(norm_h * 1080)      // h
    };

    // std::cout << "bbox: x=" << obj.rect.x
    //           << ", y=" << obj.rect.y
    //           << ", w=" << obj.rect.w
    //           << ", h=" << obj.rect.h << std::endl;

    obj.prob = 0.95f + (static_cast<float>(rand()) / RAND_MAX) * 0.05f; // 0.95-1.0
    obj.label = label;

    // 世界坐标计算
    float world_x = adjusted_x + norm_w / 2;
    float world_y = adjusted_y + norm_h / 2;

    float pos_noise = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 0.2f;
    obj.wgs84[0] = 500 + world_x * 100 + pos_noise;
    obj.wgs84[1] = 600 + world_y * 100 + pos_noise;
    obj.wgs84[2] = 200 + (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 10;

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

    // 设置激光深度
    data_pkg->set_laser_distance(50.0);

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

std::vector<std::shared_ptr<DataPackage>> generate_test_data()
{
    std::vector<std::shared_ptr<DataPackage>> test_packages;

    // 完全保持原始数据生成逻辑
    std::vector<ObjectInfo> objects1 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 2),
        CreateTestObject(0, 3, 1)};
    test_packages.push_back(CreateTestDatapackage(1634567890, 1, 0, objects1));

    std::vector<ObjectInfo> objects2 = {
        CreateTestObject(0, 1, 3),
        CreateTestObject(0, 2, 3)};
    test_packages.push_back(CreateTestDatapackage(1634567891, 2, 0, objects2));

    std::vector<ObjectInfo> objects3 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 4),
        CreateTestObject(0, 3, 3)};
    test_packages.push_back(CreateTestDatapackage(1634567892, 1, 0, objects3));

    std::vector<ObjectInfo> objects4 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 4),
        CreateTestObject(0, 3, 3)};
    test_packages.push_back(CreateTestDatapackage(1634567893, 2, 0, objects4));

    std::vector<ObjectInfo> objects5 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 4),
        CreateTestObject(0, 3, 3)};
    test_packages.push_back(CreateTestDatapackage(1634567894, 1, 0, objects5));

    std::vector<ObjectInfo> objects6 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 4),
        CreateTestObject(0, 3, 3)};
    test_packages.push_back(CreateTestDatapackage(1634567895, 2, 0, objects6));

    std::vector<ObjectInfo> objects7 = {
        CreateTestObject(0, 1, 3),
        CreateTestObject(0, 2, 1)};
    test_packages.push_back(CreateTestDatapackage(1634567896, 1, 0, objects7));

    std::vector<ObjectInfo> objects8 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 4),
        CreateTestObject(0, 3, 3)};
    test_packages.push_back(CreateTestDatapackage(1634567897, 2, 0, objects8));

    std::vector<ObjectInfo> objects9 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 4)};
    test_packages.push_back(CreateTestDatapackage(1634567898, 1, 0, objects9));

    std::vector<ObjectInfo> objects10 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 1),
        CreateTestObject(0, 3, 2)};
    test_packages.push_back(CreateTestDatapackage(1634567899, 2, 0, objects10));

    std::vector<ObjectInfo> objects11 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 2),
        CreateTestObject(0, 3, 4)};
    test_packages.push_back(CreateTestDatapackage(1634567900, 1, 0, objects11));

    std::vector<ObjectInfo> objects12 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 4),
        CreateTestObject(0, 3, 3)};
    test_packages.push_back(CreateTestDatapackage(1634567901, 2, 0, objects12));

    return test_packages;
}

// 工厂函数，根据配置创建模块实例
Module *createModule(YAML::Node config, const std::string &name, const YAML::Node &args)
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
            config["pipeline"]["stage2"]["parallel"].as<int>(),
            args["max_queue_length"].IsDefined() ? args["max_queue_length"].as<int>() : 0);
    }

    std::cerr << "Error: Unknown module type " << std::endl;
    return nullptr;
}

void setup_processing_pipeline(
    const YAML::Node &config,
    std::shared_ptr<PackageConverter> &converter,
    std::shared_ptr<Pipeline> &pipeline,
    std::shared_ptr<Fusion> &fusion,
    std::shared_ptr<Kalman> &kalman,
    std::shared_ptr<Output> &output,
    std::shared_ptr<std::vector<std::shared_ptr<DataPackage>>> input_queue, // 新增输入队列
    std::shared_ptr<std::mutex> input_lock)
{
    // 初始化数据转换模块
    const auto &input_config = config["input"]["stage2"];
    converter = std::make_shared<PackageConverter>(
        input_config["name"].as<std::string>(),
        input_queue,
        input_lock,
        input_config["args"]["max_queue_length"].as<int>());

    // 初始化处理管道
    std::vector<std::vector<Module *>> pipeline_modules;
    for (const auto &stage : config["pipeline"])
    {
        const auto &node = stage.second;
        std::vector<Module *> modules;
        for (int i = 0; i < node["parallel"].as<int>(); ++i)
        {
            modules.push_back(createModule(config, node["name"].as<std::string>(), node["args"]));
        }
        pipeline_modules.emplace_back(modules);
    }
    pipeline = std::make_shared<Pipeline>(
        pipeline_modules,
        converter->getOutputQueue(),
        converter->getOutputLock());

    // 初始化融合模块
    const auto &fusion_config = config["output"]["stage1"];
    fusion = std::make_shared<Fusion>(
        fusion_config["name"].as<std::string>(),
        fusion_config["args"]["time_slice"].as<double>(),
        pipeline->getOutputQueue(),
        pipeline->getOutputLock(),
        fusion_config["args"]["max_queue_length"].as<int>());

    // 初始化卡尔曼滤波模块
    const auto &kalman_config = config["output"]["stage2"];
    kalman = std::make_shared<Kalman>(
        kalman_config["name"].as<std::string>(),
        kalman_config["args"]["time_slice"].as<double>(),
        fusion->getOutputQueue(),
        fusion->getOutputLock(),
        kalman_config["args"]["sigma_a"].as<double>(),
        kalman_config["args"]["max_queue_length"].as<int>());
    
    // 初始传输模块
    const auto &output_config = config["output"]["stage3"];
    output = std::make_shared<Output>(
        output_config["name"].as<std::string>(),
        output_config["args"]["ip"].as<std::string>(),
        output_config["args"]["port"].as<int>(),
        output_config["args"]["interface"].as<std::string>(),
        kalman->getOutputQueue(),
        kalman->getOutputLock());

    // 启动所有处理线程
    converter->run();
    pipeline->run();
    fusion->run();
    kalman->run();
    output->run();
}

int main(int argc, char *argv[])
{
    try
    {
        // 配置初始化
        const std::string config_path = (argc > 1) ? argv[1] : "config.yaml";
        const YAML::Node config = YAML::LoadFile(config_path);

        // 创建外部输入队列和锁
        auto input_queue = std::make_shared<std::vector<std::shared_ptr<DataPackage>>>();
        auto input_lock = std::make_shared<std::mutex>();

        // 初始化处理模块
        std::shared_ptr<PackageConverter> converter;
        std::shared_ptr<Pipeline> pipeline;
        std::shared_ptr<Fusion> fusion;
        std::shared_ptr<Kalman> kalman;
        std::shared_ptr<Output> output;
        setup_processing_pipeline(config, converter, pipeline, fusion, kalman, output, input_queue, input_lock);

        // 生成测试数据
        auto test_packages = generate_test_data();

        // 输入测试数据
        std::cout << "==================== 启动流水线测试 ====================\n";
        for (const auto &data_pkg : test_packages)
        {
            {
                std::lock_guard<std::mutex> guard(*input_lock);
                input_queue->push_back(data_pkg);
            }

            std::cout << "添加数据包到输入队列，时间戳: " << data_pkg->get_timestamp()
                      << "，相机类型: " << (data_pkg->get_camera_type() == 0 ? "电视" : data_pkg->get_camera_type() == 1 ? "红外"
                                                                                                                         : "微光")
                      << "，目标数量: " << static_cast<int>(data_pkg->get_obj_num()) << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // 等待处理完成
        converter->join();
        pipeline->join();
        fusion->join();
        kalman->join();
        output->join();

        std::cout << "==================== 流水线测试完成 ====================\n";
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "配置文件错误: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "系统异常: " << e.what() << std::endl;
        return 2;
    }
    return 0;
}