#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <ctime>
#include <opencv2/opencv.hpp>

// 命名空间声明
namespace ProtocolUtils {
    // 基本类型序列化模板声明
    template <typename T>
    void serialize(std::vector<uint8_t>& buffer, T value, size_t size = sizeof(T));

    // 特化声明
    template <>
    void serialize<uint16_t>(std::vector<uint8_t>& buffer, uint16_t value, size_t size);

    template <>
    void serialize<uint32_t>(std::vector<uint8_t>& buffer, uint32_t value, size_t size);

    template <>
    void serialize<int32_t>(std::vector<uint8_t>& buffer, int32_t value, size_t size);

    template <>
    void serialize<uint64_t>(std::vector<uint8_t>& buffer, uint64_t value, size_t size);

    template <>
    void serialize<double>(std::vector<uint8_t>& buffer, double value, size_t size);

    // 反序列化模板声明
    template <typename T>
    T deserialize(const uint8_t*& data);

    template <>
    uint16_t deserialize<uint16_t>(const uint8_t*& data);

    template <>
    uint32_t deserialize<uint32_t>(const uint8_t*& data);

    template <>
    int32_t deserialize<int32_t>(const uint8_t*& data);

    template <>
    uint64_t deserialize<uint64_t>(const uint8_t*& data);

    template <>
    double deserialize<double>(const uint8_t*& data);

    // OpenCV矩阵序列化/反序列化声明
    void serializeMat(std::vector<uint8_t>& buffer, const cv::Mat& img);
    cv::Mat deserializeMat(const uint8_t*& data);
}

// 包含模板定义
#include "utils/protocol.inl"