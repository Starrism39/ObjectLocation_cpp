#include <iostream>
#include <string>
#include <vector>
#include <memory>

// TODO：输入、输出
#include "modules/output.h"
#include "modules/converter.h"

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

void printPackagesDP(const std::vector<Package>& pkgs) {
    std::cout << "Total packages: " << pkgs.size() << std::endl;
    for(size_t i = 0; i < pkgs.size(); i++) {
        std::cout << "Package " << i << ":" << std::endl;
        std::cout << "  time: " << pkgs[i].time << std::endl;
        std::cout << "  uav_id: " << pkgs[i].uav_id << std::endl;
        std::cout << "  Bbox size: " << pkgs[i].Bbox.size() << std::endl;
        std::cout << "  location size: " << pkgs[i].location.size() << std::endl;
        if(pkgs[i].dp) {
            std::cout << "  dp pointer: " << pkgs[i].dp.get() << std::endl;
            std::cout << "  dp use count: " << pkgs[i].dp.use_count() << std::endl;
        } else {
            std::cout << "  dp is null" << std::endl;
        }
        std::cout << std::endl;
    }
}




int main(){
    Output* fusion = new Output(1000);

    // 创建测试数据包
    std::shared_ptr<DataPackage> test_package;

    // 场景1：电视相机，多个目标
    std::vector<ObjectInfo> objects1 = {
        CreateTestObject(0, 1, 1),
        CreateTestObject(0, 2, 2),
        CreateTestObject(0, 3, 1)};
    test_package = CreateTestDatapackage(1634567890, 1, 0, objects1);
    
    std::vector<Package> pkgs = PackageConverter::ConvertToPackages(test_package);

    std::cout << "转换为package成功" << std::endl;
    std::cout << "融合前的目标数量为" << static_cast<int>(test_package->get_obj_num()) << std::endl;
    
    for(auto & pkg : pkgs){
        pkg.location = {pkg.uav_utm[0], pkg.uav_utm[1], pkg.uav_utm[2]};
    }
    std::shared_ptr<DataPackage> result = fusion->process(pkgs);
    std::cout << "融合前的目标数量为" << static_cast<int>(result->get_obj_num()) << std::endl;
    
}