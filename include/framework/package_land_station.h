#pragma once
#include "framework/package.h"
#include <functional>

class DataPackageLandStation : public DataPackage
{
public:
    DataPackageLandStation() : DataPackage() {};
    ~DataPackageLandStation() {};
    const uint8_t *get_location_stream() const;
    const size_t get_location_stream_length() const;
    void encoder_stream_to_location();
    void parse_stream_location(uint8_t *stream, const size_t len);
    void set_jpeg_quality(int jpeg_quality);

private:
    size_t location_stream_len_ = 0;             // 传输给定位模块码流长度
    std::unique_ptr<uint8_t[]> location_stream_; // 传输给定位模块码流
    int jpeg_quality_ = 50;                      // 压缩质量，0-100
    std::vector<uint8_t> img_encoder(const cv::Mat &img)
    {
        std::vector<uint8_t> jpeg_buffer;
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpeg_quality_); // 压缩质量，0-100
        cv::imencode(".jpg", img, jpeg_buffer, compression_params);
        return jpeg_buffer;
    };

    cv::Mat img_decoder(const std::vector<uint8_t> &jpeg_data)
    {
        cv::Mat img = cv::imdecode(jpeg_data, cv::IMREAD_GRAYSCALE);
        return img;
    };
};