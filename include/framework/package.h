/*************************************************************************************************************************
 * Copyright 2025 Grifcc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/
#pragma once

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include "framework/data_object.h"

//  0电视: 1280x1080
//  1红外: 640x512
//  2微光: 1280x1080
// const int img_size[3][2] = {{1280, 1080}, {640, 512}, {1280, 1080}};
const int img_size[3][2] = {{1920, 1080}, {640, 512}, {1920, 1080}};

struct Bbox
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
};

// 目标信息
struct ObjectInfo
{
    /* data */
    uint8_t uid;
    uint16_t tracker_id;
    Bbox rect;
    float prob;
    uint8_t label;
    double wgs84[3]; // 经纬度
};

// 相机矩阵，6个double，48byte
typedef union
{
    struct
    {
        double x;
        double y;
        double z;
        double yaw;
        double pitch;
        double roll;
    } member;
    uint8_t data[48];
} CameraMatrix;

// 帧头，8bit
struct CameraInfo
{
    uint8_t uav_id;      // 有效低4bit, 无人机id, 0-15
    uint8_t camera_type; // 有效低2bit, 相机类型, 0:电视 1:红外 2:微光
    CameraMatrix cm;     // 相机矩阵
};

// 单应矩阵，9个float，36byte
typedef union
{
    float h[9];
    uint8_t data[36];
} Homography;

// 数据结构
class DataPackage : public GryFlux::DataObject
{
public:
    DataPackage() : align_bits_(3)
    {
        this->clear();
    };
    ~DataPackage() {};

    // 机上
    // 源
    void set_timestamp(uint64_t timestamp); // 设置时间戳
    void set_camera_info(const uint8_t uav_id, const uint8_t camera_type, const uint8_t *cm_data);
    void set_camera_info(const uint8_t uav_id, const uint8_t camera_type, const double *cm); // 设置相机信息
    void set_laser_distance(float laser_distance);                                           // 设置激光距离

    // 目标检测
    void set_obj_num(uint8_t obj_num);            // 设置目标数量
    void push_object_info(const ObjectInfo &obj); // 添加目标信息
    // xfeat
    void set_homography(const uint8_t *h); // 设置单应矩阵
    void set_homography(const float *h);   // 设置单应矩阵

    // encoder
    void set_imgcode(const uint8_t *img_code, const size_t len);      // 设置图像码流
    void set_foreground(const uint8_t *foreground, const size_t len); // 设置前景流(bbox&bbox[i-1])
    void set_background(const uint8_t *background, const size_t len); // 设置背景流
    void set_keyframe(bool key_frame);                                // 设置关键帧
    void set_stream(const uint8_t *stream, const size_t len); // 设置码流

    // 机上几下均有
    void set_rgb(const cv::Mat &rgb);           // 设置rgb图像，源和decoder
    void set_ir(const cv::Mat &ir);             // 设置ir图像， 源和decoder
    void set_lowlight(const cv::Mat &lowlight); // 设置lowlight图像，源和decoder

    const cv::Mat &get_rgb() const;      // 获取rgb图像，目标检测，encoder，xfeat，display
    const cv::Mat &get_ir() const;       // 获取ir图像, 目标检测，encoder，xfeat，display
    const cv::Mat &get_lowlight() const; // 获取lowlight图像, 目标检测，encoder，xfeat，display

    void encoder_stream();

    // 机下
    void parse_stream(); // 解析流(recv module 包含熵编码的decoder)

    uint64_t get_timestamp() const;                  // 获取时间戳， location, display
    bool is_keyframe() const;                        // 判断是否为关键帧, decoder
    uint8_t get_uav_id() const;                      // 获取无人机id， location
    uint8_t get_camera_type() const;                 // 获取相机类型， location
    Homography get_homography() const;               // 获取单应矩阵， decoder
    CameraMatrix get_camera_matrix() const;          // 获取相机矩阵， location
    uint8_t get_obj_num() const;                     // 获取目标数量， location，decoder，dispaly
    std::vector<ObjectInfo> get_object_info() const; // 获取目标信息， location，decoder，dispaly
    float get_laser_distance() const;                // 获取激光距离， location

    // send module(包含熵编码的encoder)
    const uint8_t *get_stream() const;
    const size_t get_stream_length() const;

    const uint8_t *get_imgcode() const;
    const size_t get_imgcode_length() const;
    const uint8_t *get_foreground() const;
    const size_t get_foreground_length() const;
    const uint8_t *get_background() const;
    const size_t get_background_length() const;

    void clear(); // 清空数据
private:
    const uint32_t align_bits_; // 对齐位数

    cv::Mat rgb_;
    cv::Mat ir_;
    cv::Mat lowlight_;

    uint64_t timestamp_;     // 时间戳
    Homography homography_;  // 单应矩阵
    CameraInfo camera_info_; // 相机矩阵
    bool key_frame_;         // 关键帧
    float laser_distance_;   // 激光距离

    uint8_t obj_num_;                     // 目标数量
    std::vector<ObjectInfo> object_info_; // 目标信息

    size_t stream_len_ = 0;             // 码流长度
    std::unique_ptr<uint8_t[]> stream_; // 码流

    size_t imgcode_len_ = 0;             // 图像码流长度
    std::unique_ptr<uint8_t[]> imgcode_; // 图像码流

    size_t foreground_len_ = 0;             // 前景长度
    std::unique_ptr<uint8_t[]> foreground_; // 前景

    size_t background_len_ = 0;             // 背景长度
    std::unique_ptr<uint8_t[]> background_; // 背景

private:
    Bbox quant_bbox(const Bbox bbox, const uint32_t align_bits = 3) const;   //  xywh坐标量化
    Bbox align_bbox(const Bbox bbox, const uint32_t align_bits = 3) const;   //  xywh坐标对齐,默认值为3，即对齐到8
    Bbox unalign_bbox(const Bbox bbox, const uint32_t align_bits = 3) const; //  xywh坐标解对齐

    void pack_objinfo(const ObjectInfo &obj, uint8_t *data);
    void unpack_objinfo(const uint8_t *data, ObjectInfo &obj);
};
