#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "yaml-cpp/yaml.h"

#include"framework/pipeline.h"

// TODO：输入、数据预处理、输出
#include"modules/time_filter.h"
#include"modules/esti_position.h"
#include"modules/spatial_filter.h"

// 工厂函数，根据配置创建模块实例
Module* createModule(const std::string& name, const YAML::Node& args) {

    if (name == "TimeFilter") {
        return new TimeFilter(
            args["time_slice"].as<double>(),
            args["max_queue_length"].IsDefined() ? 
                args["max_queue_length"].as<int>() : 0
        );
    }

    else if (name == "EstiPosition") {
        if (args["is_multi_map"].as<bool>()) {
            // 多地图版本
            return new EstiPosition(
                args["is_multi_map"].as<bool>(),
                args["map_path"].as<std::string>(),
                args["default_height"].as<double>(),
                args["order"].as<std::string>(),
                args["enable_reloaction"].as<bool>(),
                args["max_queue_length"].IsDefined() ? 
                    args["max_queue_length"].as<int>() : 0,
                args["multi_num"].as<int>(),
                [&args]() {
                    std::vector<double> scale;
                    for (const auto& val : args["downSampled_scale"]) {
                        scale.push_back(val.as<double>());
                    }
                    return scale;
                }(),
                [&args]() {
                    std::vector<std::vector<double>> scale;
                    for (const auto& outer : args["split_scale"]) {
                        std::vector<double> inner;
                        for (const auto& val : outer) {
                            inner.push_back(val.as<double>());
                        }
                        scale.push_back(inner);
                    }
                    return scale;
                }(),
                args["block_num"].as<int>(),
                args["overlap_x_scale"].as<double>(),
                args["overlap_y_scale"].as<double>()
            );
        } else {
            // 单地图版本
            return new EstiPosition(
                args["is_multi_map"].as<bool>(),
                args["map_path"].as<std::string>(),
                args["default_height"].as<double>(),
                args["order"].as<std::string>(),
                args["enable_reloaction"].as<bool>(),
                args["max_queue_length"].IsDefined() ? 
                    args["max_queue_length"].as<int>() : 0
            );
        }
    }

    else if (name == "SpatialFilter") {
        return new SpatialFilter(
            args["time_slice"].as<double>(),
            args["distance_threshold"].as<double>(),
            args["max_map"].as<int>(),
            args["max_queue_length"].IsDefined() ? 
                args["max_queue_length"].as<int>() : 0
        );
    }

    
    throw std::runtime_error("Unknown module type: " + name);
}

int main(int argc, char* argv[]) {
    // 设置默认配置文件路径
    std::string config_path = "config.yaml";
    
    // 如果提供了命令行参数，使用指定的配置文件
    if (argc > 1) {
        config_path = argv[1];
    }
    
    try {
        // 读取YAML配置文件
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 存储所有管道阶段的模块
        std::vector<std::vector<Module*>> pipelines;
        
        // 遍历pipeline节点
        for (const auto& stage : config["pipeline"]) {
            std::vector<Module*> stage_modules;
            
            // 获取模块名称和参数
            std::string module_name = stage.second["name"].as<std::string>();
            YAML::Node args = stage.second["args"];
            int parallel = stage.second["parallel"].as<int>();
            
            // 创建指定数量的并行模块
            for (int i = 0; i < parallel; ++i) {
                Module* module = createModule(module_name, args);
                stage_modules.push_back(module);
            }
            
            pipelines.push_back(stage_modules);
        }
        
        // 创建并运行管道
        Pipeline pipeline(pipelines);
        pipeline.join();
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}