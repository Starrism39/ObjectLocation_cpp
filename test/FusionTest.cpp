#include <vector>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <memory>

#include "modules/fusion.h"

void test_fusion()
{
    // 准备测试数据
    std::vector<Package> pkgs;

    // 使用构造函数创建Package对象
    auto createPackage = [](time_t t, const char *uav_id,
                            std::vector<int> bbox,
                            std::vector<double> loc,
                            int gid)
    {
        Package pkg(t);
        pkg.uav_id = uav_id;
        pkg.Bbox = bbox;
        pkg.location = loc;
        pkg.global_id = gid;

        // 必须初始化的其他字段
        pkg.camera_pose = {0, 0, 0, 0, 0, 0};
        pkg.camera_K = {0, 0, 0, 0};
        pkg.camera_distortion = {0, 0, 0, 0, 0};
        pkg.norm_Bbox = {0, 0, 0, 0};
        pkg.prob = 1.0f;
        pkg.class_id = 0;
        pkg.class_name = "test";
        pkg.tracker_id = 0;
        pkg.dp = std::make_shared<DataPackage>();

        return pkg;
    };

    pkgs.push_back(createPackage(
        1634567890, "1",
        {100, 200, 300, 400},
        {591.74, 682.426, 150},
        1));

    pkgs.push_back(createPackage(
        1634567890, "1",
        {300, 210, 500, 200},
        {600, 652.426, 150},
        2));
    
    pkgs.push_back(createPackage(
        1634567890, "1",
        {971, 554, 384, 324},
        {500, 690, 150},
        3));

    pkgs.push_back(createPackage(
        1634567891, "2",
        {150, 250, 350, 450},
        {591.74, 682.426, 150},
        1));

    pkgs.push_back(createPackage(
        1634567891, "2",
        {900, 450, 400, 300},
        {500, 690, 150},
        3));

    pkgs.push_back(createPackage(
        1634567891, "2",
        {100, 200, 200, 300},
        {587, 697, 120},
        4));

    // 执行融合
    OutPackage result = Fusion::fusion(pkgs);

    // 验证结果
    std::cout << "\n=== 测试结果 ===" << std::endl;

    printOutPackage(result);
}

int main()
{
    test_fusion();
    return 0;
}
