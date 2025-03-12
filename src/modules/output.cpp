#include <iostream>
#include <algorithm>

#include "modules/output.h"


void Output::UTMToWGS84(const std::vector<double>& utm, double wgs84[3]) {
    // UTM参数
    const double k0 = 0.9996;                    // UTM比例因子
    const double a = 6378137.0;                  // WGS84椭球体长半轴
    const double e = 0.081819190842622;          // WGS84椭球体偏心率
    const double e2 = e * e;
    const double e_p2 = e2 / (1.0 - e2);
    const double e1 = (1 - sqrt(1 - e2)) / (1 + sqrt(1 - e2));  // 第一偏心率

    
    // 从UTM坐标中获取东向位置和北向位置（假设单位是米）
    double x = utm[0] - 500000.0;  // 东向位置（从中央经线开始）
    double y = utm[1];             // 北向位置
    double z = utm.size() > 2 ? utm[2] : 0.0;   // 高度（如果有）

    // 计算中央经线的经度（需要知道UTM带号）
    // 这里假设UTM带号是31（覆盖经度0°到6°）
    // 实际使用时需要根据具体位置确定带号
    int zone = 31;  // 需要根据实际情况修改
    double lon0 = (zone - 1) * 6 - 180 + 3;  // 中央经线经度（度）
    lon0 = lon0 * M_PI / 180.0;              // 转换为弧度

    // 计算参数
    double M = y / k0;
    double mu = M / (a * (1 - e2/4 - 3*e2*e2/64 - 5*e2*e2*e2/256));
    
    double phi1 = mu + (3*e1/2 - 27*e1*e1*e1/32)*sin(2*mu) 
                    + (21*e1*e1/16 - 55*e1*e1*e1*e1/32)*sin(4*mu)
                    + (151*e1*e1*e1/96)*sin(6*mu)
                    + (1097*e1*e1*e1*e1/512)*sin(8*mu);
    
    double N1 = a / sqrt(1 - e2*sin(phi1)*sin(phi1));
    double T1 = tan(phi1)*tan(phi1);
    double C1 = e_p2*cos(phi1)*cos(phi1);
    double R1 = a*(1-e2)/pow(1-e2*sin(phi1)*sin(phi1), 1.5);
    double D = x/(N1*k0);

    // 计算纬度
    double lat = phi1 - (N1*tan(phi1)/R1)*(D*D/2
                 - (5 + 3*T1 + 10*C1 - 4*C1*C1 - 9*e_p2)*D*D*D*D/24
                 + (61 + 90*T1 + 298*C1 + 45*T1*T1 - 252*e_p2 - 3*C1*C1)*D*D*D*D*D*D/720);

    // 计算经度
    double lon = lon0 + (D - (1 + 2*T1 + C1)*D*D*D/6
                 + (5 - 2*C1 + 28*T1 - 3*C1*C1 + 8*e_p2 + 24*T1*T1)
                 *D*D*D*D*D/120)/cos(phi1);

    // 转换为度
    wgs84[0] = lon * 180.0 / M_PI;  // 经度
    wgs84[1] = lat * 180.0 / M_PI;  // 纬度
    wgs84[2] = z;                    // 高度
}

Output::Output(int max_queue_length)
    :Sink::Sink("Output", max_queue_length){}

std::shared_ptr<DataPackage> Output::process(const std::vector<Package>& packages){
    uint8_t num = 0;
    for(const auto& pkg : packages){
        Bbox rect{
            static_cast<uint16_t>(pkg.Bbox[0]), 
            static_cast<uint16_t>(pkg.Bbox[1]), 
            static_cast<uint16_t>(pkg.Bbox[2]), 
            static_cast<uint16_t>(pkg.Bbox[3])
        };
        double wgs84[3];
        UTMToWGS84(pkg.location, wgs84);
        ObjectInfo obj{
            static_cast<uint8_t>(pkg.global_id), 
            static_cast<uint16_t>(pkg.tracker_id), 
            rect, 
            pkg.prob, 
            static_cast<uint8_t>(pkg.class_id), 
            {wgs84[0], wgs84[1], wgs84[2]}
        };
        pkg.dp->push_object_info(obj);
        num++;
    }
    packages[0].dp->set_obj_num(num);
    return packages[0].dp;
}
