#include "utils/package_land_station.h"
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
    // 1. 初始化空间
    cv::Mat rgb = this->get_rgb();
    int len_location_info = 8 + 1 + 36 + 48 + 2 + 1 + this->get_obj_num() * 7;
    int len_reserved = 2;
    this->location_stream_len_ = len_location_info + len_reserved + rgb.cols * rgb.rows; //  信息体 : 时间8B +
                                                                                         //  相机信息1B(无人机ID 4bit+相机类型2bit+帧类型2bit)
                                                                                         //  H矩阵36B+相机绝对位姿48B
                                                                                         //  激光距离2B
                                                                                         //  目标数量1B
                                                                                         //  目标信息 7*目标数量B
                                                                                         //  背景大小2B
                                                                                         //  图像数据
    this->location_stream_ = std::make_unique<uint8_t[]>(this->location_stream_len_);

    // 2. 拼包
    size_t offset = 0;
    this->encoder_stream();
    const uint8_t *stream = this->get_stream();
    memcpy(this->location_stream_.get() + offset, stream, len_location_info);
    offset += len_location_info;

    // 3.reserved
    uint16_t background_len = 0;
    memcpy(this->location_stream_.get() + offset, &background_len, 2);
    offset += 2;

    // 图像数据
    std::vector<uchar> jpeg_buffer = img_encoder(rgb);
    memcpy(this->location_stream_.get() + offset, (uint8_t *)jpeg_buffer.data(), jpeg_buffer.size());
    this->location_stream_len_ = len_location_info + len_reserved + jpeg_buffer.size();
}

void DataPackageLandStation::parse_stream_location(uint8_t *stream, const size_t len)
{
    size_t offset = 0;
    // 目标数量
    uint8_t obj_num;
    memcpy(&obj_num, stream + 95, 1);
    int len_location_info = 8 + 1 + 36 + 48 + 2 + 1 + obj_num * 7;
    this->parse_stream(stream, len_location_info + 2);
    offset += (len_location_info + 2);

    // 图像数据
    std::vector<uchar> jpeg_data = std::vector<uchar>(stream + offset, stream + len);
    cv::Mat img = img_decoder(jpeg_data);
    cv::Mat rgb = cv::Mat(1080, 1280, CV_8UC1);
    memcpy(rgb.data, img.data, img.cols * img.rows);
    this->set_rgb(rgb);
}