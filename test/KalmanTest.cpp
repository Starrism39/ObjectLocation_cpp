#include <vector>
#include <iostream>
#include "modules/kalman.h"

// 测试场景1：单目标两帧连续观测
void testTwoFrames()
{
    std::cout << "==== 测试场景：匀速运动目标 ====\n";

    // 生成测试数据（按时间降序排列）
    std::vector<OutPackage> packages;

    // 第二帧数据（时间靠后）
    {
        OutPackage pkg;
        pkg.time = 1; // 较晚的时间戳
        Object obj;
        obj.global_id = 1;
        obj.location = {2.0, 1.0, 0.0}; // 第二帧位置
        pkg.objs.push_back(obj);
        packages.push_back(pkg);
    }

    // 第一帧数据（时间较早）
    {
        OutPackage pkg;
        pkg.time = 0; // 较早的时间戳
        Object obj;
        obj.global_id = 1;
        obj.location = {0.0, 0.0, 0.0}; // 第一帧位置
        pkg.objs.push_back(obj);
        packages.push_back(pkg);
    }

    // 执行卡尔曼滤波（过程噪声参数设为0.1）
    OutPackage result = Kalman::kalman(packages, 0.1f);

    printOutPackage(result);
}

// 测试场景2：多目标不同步观测
void testMultipleTargets()
{
    std::cout << "==== 测试场景：多目标观测 ====\n";

    std::vector<OutPackage> packages;

    // 第三帧（最新）
    {
        OutPackage pkg;
        pkg.time = 2;
        // 目标1存在
        Object obj1;
        obj1.global_id = 1;
        obj1.location = {3.0, 3.0, 0.0};
        // 目标2新出现
        Object obj2;
        obj2.global_id = 2;
        obj2.location = {5.0, 0.0, 0.0};
        pkg.objs = {obj1, obj2};
        packages.push_back(pkg);
    }

    // 第二帧
    {
        OutPackage pkg;
        pkg.time = 1;
        // 目标1存在
        Object obj1;
        obj1.global_id = 1;
        obj1.location = {2.0, 1.0, 0.0};
        pkg.objs = {obj1};
        packages.push_back(pkg);
    }

    // 第一帧（最早）
    {
        OutPackage pkg;
        pkg.time = 0;
        // 目标1存在
        Object obj1;
        obj1.global_id = 1;
        obj1.location = {0.0, 0.0, 0.0};
        pkg.objs = {obj1};
        packages.push_back(pkg);
    }

    OutPackage result = Kalman::kalman(packages, 0.1f);
    printOutPackage(result);
}

int main()
{
    // 运行测试场景
    testTwoFrames();
    testMultipleTargets();

    return 0;
}
