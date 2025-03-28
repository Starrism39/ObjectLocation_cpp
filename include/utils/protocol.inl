#pragma once
#include <cstring>
#include <arpa/inet.h> // 字节序转换

namespace ProtocolUtils {

// 基本类型序列化模板定义
template <typename T>
void serialize(std::vector<uint8_t>& buffer, T value, size_t size) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), bytes, bytes + size);
}

// 专用化处理不同字节序类型
template <>
inline void serialize<uint16_t>(std::vector<uint8_t>& buffer, uint16_t value, size_t /*size*/) {
    value = htons(value);
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), bytes, bytes + sizeof(uint16_t));
}

template <>
inline void serialize<uint32_t>(std::vector<uint8_t>& buffer, uint32_t value, size_t /*size*/) {
    value = htonl(value);
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), bytes, bytes + sizeof(uint32_t));
}

template <>
inline void serialize<int32_t>(std::vector<uint8_t>& buffer, int32_t value, size_t /*size*/){
    value = htonl(static_cast<uint32_t>(value));
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), bytes, bytes + sizeof(uint32_t));
}

template <>
inline void serialize<uint64_t>(std::vector<uint8_t>& buffer, uint64_t value, size_t /*size*/){
    value = htobe64(value);
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), bytes, bytes + sizeof(uint64_t));
}

template <>
inline void serialize<double>(std::vector<uint8_t>& buffer, double value, size_t /*size*/) {
    uint64_t temp;
    static_assert(sizeof(double) == sizeof(uint64_t), "Size mismatch");
    memcpy(&temp, &value, sizeof(double));
    temp = htobe64(temp);
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&temp);
    buffer.insert(buffer.end(), bytes, bytes + sizeof(uint64_t));
}

// 反序列化模板定义
template <typename T>
T deserialize(const uint8_t*& data) {
    T value;
    memcpy(&value, data, sizeof(T));
    data += sizeof(T);
    return value;
}

template <>
inline uint16_t deserialize<uint16_t>(const uint8_t*& data) {
    uint16_t raw;
    memcpy(&raw, data, sizeof(uint16_t));
    data += sizeof(uint16_t);
    return ntohs(raw); 
}

template <>
inline uint32_t deserialize<uint32_t>(const uint8_t*& data) {
    uint32_t raw;
    memcpy(&raw, data, sizeof(uint32_t));
    data += sizeof(uint32_t);
    return ntohl(raw);
}

template <>
inline int32_t deserialize<int32_t>(const uint8_t*& data){
    uint32_t raw;
    memcpy(&raw, data, sizeof(uint32_t));
    data += sizeof(uint32_t);
    return static_cast<int32_t>(ntohl(raw));
}

template <>
inline uint64_t deserialize<uint64_t>(const uint8_t*& data){
    uint64_t raw;
    memcpy(&raw, data, sizeof(uint64_t));
    data += sizeof(uint64_t);
    return be64toh(raw);
}

template <>
inline double deserialize<double>(const uint8_t*& data) {
    uint64_t raw;
    memcpy(&raw, data, sizeof(uint64_t)); 
    data += sizeof(uint64_t);
    raw = be64toh(raw);
    double value;
    memcpy(&value, &raw, sizeof(double));
    return value;
}

} // namespace ProtocolUtils
