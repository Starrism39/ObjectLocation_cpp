#include "framework/package_land_station.h"
#include "utils/two_byte_float.h"

const uint8_t *DataPackageLandStation::get_location_stream() const
{
    return this->location_stream_.get();
}

const size_t DataPackageLandStation::get_location_stream_length() const
{
    return this->location_stream_len_;
}

void DataPackageLandStation::set_jpeg_quality(int jpeg_quality)
{
    this->jpeg_quality_ = jpeg_quality;
}

void DataPackageLandStation::encoder_stream_to_location()
{
    // 1. 获取图像码流
    std::vector<uint8_t> jpeg_buffer = img_encoder(this->get_rgb());
    uint32_t jpg_size = static_cast<uint32_t>(jpeg_buffer.size());

    // 2. 获取目标信息和相机信息
    const uint8_t *stream = this->get_stream();
    size_t header_len = this->get_stream_length() - this->get_background_length() - this->get_foreground_length() - 2; // 原始码流减去背景和前景码流等于目标信息和相机信息

    // 3. 计算码流大小，等于目标信息和相机信息长度+2B背景长度+4B图像码流长度+图像码流
    this->location_stream_len_ = header_len + sizeof(uint16_t) + sizeof(uint32_t) + jpeg_buffer.size();

    // 4. 初始化码流
    this->location_stream_ = std::make_unique<uint8_t[]>(this->location_stream_len_);

    // 5. 拼接目标信息和相机信
    size_t offset = 0;
    memcpy(this->location_stream_.get() + offset, stream, header_len);
    offset += header_len;

    // 6. 置原始背景长度为0
    uint16_t background_len = 0;
    memcpy(this->location_stream_.get() + offset, &background_len, sizeof(uint16_t));
    offset += sizeof(uint16_t);

    // 7. 拼接图像码流长度
    memcpy(this->location_stream_.get() + offset, &jpg_size, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // 8. 拼接图像码流
    memcpy(this->location_stream_.get() + offset, (uint8_t *)jpeg_buffer.data(), jpg_size);
    offset += jpg_size;
}

void DataPackageLandStation::parse_stream_location(uint8_t *stream, const size_t len)
{
    size_t offset = 0;
    // 1. 解析目标信息和相机信息
    size_t camera_info_len = 8 + 1 + 36 + 48 + 2;
    size_t obj_info_len = *reinterpret_cast<uint8_t *>(stream + camera_info_len) * 7;
    size_t header_len = camera_info_len + 1 + obj_info_len + 2; // 相机信息长度+ 1B + 目标信息长度 + 2B

    this->set_stream(stream, header_len);
    this->parse_stream();
    offset += header_len;

    // 2. 解析图像码流长度
    uint32_t jpg_size;
    memcpy(&jpg_size, stream + offset, sizeof(uint32_t));
    offset += sizeof(uint32_t);
    // std::cout << "jpg size: " << jpg_size << std::endl;

    // 3. 解析图像码流
    std::vector<uint8_t> jpeg_data = std::vector<uint8_t>(stream + offset, stream + offset + jpg_size);
    this->set_rgb(img_decoder(jpeg_data));
    offset += jpg_size;
}