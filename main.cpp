#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "yaml-cpp/yaml.h"

#include "framework/pipeline.h"

#include "input/input.h"
#include "input/converter.h"

#include "modules/time_filter.h"
#include "modules/esti_position.h"
#include "modules/PkgArrange.h"
#include "modules/spatial_filter.h"
#include "modules/RTK_difference.h"

#include "output/fusion.h"
#include "output/kalman.h"
#include "output/output.h"

// 工厂函数，根据配置创建管道模块实例
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

    else if (name == "PkgArrange")
    {
        return new PkgArrange(
            args["max_queue_length"].IsDefined() ? args["max_queue_length"].as<int>() : 0);
    }

    else if (name == "RTKDifference")
    {
        return new RTKDifference(
            args["time_slice"].as<double>(),
            args["class_1"].as<int>(),
            args["x1"].as<double>(),
            args["y1"].as<double>(),
            args["max_queue_length"].IsDefined() ? args["max_queue_length"].as<int>() : 0);
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
    std::shared_ptr<Input> &input,
    std::shared_ptr<PackageConverter> &converter,
    std::shared_ptr<Pipeline> &pipeline,
    std::shared_ptr<Fusion> &fusion,
    std::shared_ptr<Kalman> &kalman,
    std::shared_ptr<Output> &output)
{
    // 初始化数据输入模块
    const auto &input_config = config["input"]["stage1"];
    std::string endpoint1 = input_config["args"]["endpoint1"].as<std::string>();
    std::string endpoint2 = input_config["args"]["endpoint2"].as<std::string>();
    std::vector<std::string> endpoints = {endpoint1, endpoint2};
    std::cout << "\nBuilding " << input_config["name"].as<std::string>() << std::endl;
    input = std::make_shared<Input>(
        input_config["name"].as<std::string>(),
        endpoints,
        input_config["args"]["topic"].as<std::string>());

    // 初始化数据转换模块
    const auto &convert_config = config["input"]["stage2"];
    converter = std::make_shared<PackageConverter>(
        convert_config["name"].as<std::string>(),
        input->getOutputQueue(),
        input->getOutputLock(),
        convert_config["args"]["uav_id"].as<int>(),
        convert_config["args"]["del_easting"].as<double>(),
        convert_config["args"]["del_northing"].as<double>(),
        convert_config["args"]["del_uav1_height"].as<double>(),
        convert_config["args"]["del_uav2_height"].as<double>(),
        convert_config["args"]["max_queue_length"].as<int>());

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
    input->run();
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

        // 初始化处理模块
        std::shared_ptr<Input> input;
        std::shared_ptr<PackageConverter> converter;
        std::shared_ptr<Pipeline> pipeline;
        std::shared_ptr<Fusion> fusion;
        std::shared_ptr<Kalman> kalman;
        std::shared_ptr<Output> output;
        setup_processing_pipeline(config, input, converter, pipeline, fusion, kalman, output);

        // 等待处理完成
        input->join();
        converter->join();
        pipeline->join();
        fusion->join();
        kalman->join();
        output->join();
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