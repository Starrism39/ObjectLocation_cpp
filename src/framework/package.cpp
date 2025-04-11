#include "framework/package.h"
#include "utils/two_byte_float.h"

void DataPackage::set_timestamp(uint64_t timestamp)
{
    this->timestamp_ = timestamp;
}

void DataPackage::set_camera_info(const uint8_t uav_id, const uint8_t camera_type, const uint8_t *cm_data)
{
    this->camera_info_.uav_id = uav_id;
    this->camera_info_.camera_type = camera_type;
    memcpy(this->camera_info_.cm.data, cm_data, sizeof(uint8_t) * 48);
}

void DataPackage::set_camera_info(const uint8_t uav_id, const uint8_t camera_type, const double *cm)
{
    this->camera_info_.uav_id = uav_id;
    this->camera_info_.camera_type = camera_type;
    this->camera_info_.cm.member.x = cm[0];
    this->camera_info_.cm.member.y = cm[1];
    this->camera_info_.cm.member.z = cm[2];
    this->camera_info_.cm.member.yaw = cm[3];
    this->camera_info_.cm.member.pitch = cm[4];
    this->camera_info_.cm.member.roll = cm[5];
}

void DataPackage::set_laser_distance(float laser_distance)
{
    this->laser_distance_ = laser_distance;
}

void DataPackage::set_obj_num(uint8_t obj_num)
{
    this->obj_num_ = obj_num;
}

void DataPackage::push_object_info(const ObjectInfo &obj)
{
    // 量化bbox
    auto quanted_bbox = this->quant_bbox(obj.rect, this->align_bits_);
    ObjectInfo quanted_obj;
    quanted_obj.uid = obj.uid;
    quanted_obj.tracker_id = obj.tracker_id;
    quanted_obj.rect = quanted_bbox;
    quanted_obj.prob = obj.prob;
    quanted_obj.label = obj.label;
    quanted_obj.wgs84[0] = obj.wgs84[0];
    quanted_obj.wgs84[1] = obj.wgs84[1];
    quanted_obj.wgs84[2] = obj.wgs84[2];

    this->object_info_.push_back(quanted_obj);
}

void DataPackage::set_homography(const uint8_t *h_data)
{
    memcpy(this->homography_.data, h_data, sizeof(uint8_t) * 36);
}

void DataPackage::set_homography(const float *h)
{
    memcpy(this->homography_.h, h, sizeof(float) * 9);
}

void DataPackage::set_imgcode(const uint8_t *img_code, const size_t len)
{
    this->imgcode_len_ = len;
    this->imgcode_ = std::make_unique<uint8_t[]>(len);
    memcpy(this->imgcode_.get(), img_code, len);
}

void DataPackage::set_foreground(const uint8_t *foreground, const size_t len)
{
    this->foreground_len_ = len;
    this->foreground_ = std::make_unique<uint8_t[]>(len);
    memcpy(this->foreground_.get(), foreground, len);
}

void DataPackage::set_background(const uint8_t *background, const size_t len)
{
    this->background_len_ = len;
    this->background_ = std::make_unique<uint8_t[]>(len);
    memcpy(this->background_.get(), background, len);
}

void DataPackage::set_keyframe(bool key_frame)
{
    this->key_frame_ = key_frame;
}

void DataPackage::set_rgb(const cv::Mat &rgb)
{
    this->rgb_ = rgb;
}

void DataPackage::set_ir(const cv::Mat &ir)
{
    this->ir_ = ir;
}

void DataPackage::set_lowlight(const cv::Mat &lowlight)
{
    this->lowlight_ = lowlight;
}

const cv::Mat &DataPackage::get_rgb() const
{
    return this->rgb_;
}

const cv::Mat &DataPackage::get_ir() const
{
    return this->ir_;
}

const cv::Mat &DataPackage::get_lowlight() const
{
    return this->lowlight_;
}

void DataPackage::encoder_stream()
{
    // 1. 熵编码 encoder
    // size_t encodered_foreground_len;
    // uint8_t *encodered_foreground;
    // compress_stream(this->foreground_, this->foreground_len_,encodered_foreground, encodered_foreground_len)
    // size_t encodered_background_len;
    // uint8_t *encodered_background;
    // compress_stream(this->background_,this->background_len_,encodered_background, encodered_background_len)

    // 2. 初始化空间
    this->stream_len_ = 8 + 1 + 36 + 48 + 2 + 1 + this->obj_num_ * 7 + 2 + background_len_ + foreground_len_; //  信息体 : 时间8B +
                                                                                                              //  相机信息1B(无人机ID 4bit+相机类型2bit+帧类型2bit)
                                                                                                              //  H矩阵36B+相机绝对位姿48B
                                                                                                              //  激光距离2B
                                                                                                              //  目标数量1B
                                                                                                              //  目标信息 7*目标数量B
                                                                                                              //  背景大小2B
    //  this->stream_len_ += encoder_background_len;
    //  this->stream_len_ += encoder_foreground_len;
    this->stream_ = std::make_unique<uint8_t[]>(this->stream_len_);

    // 3. 拼包
    size_t offset = 0;

    // 时间戳
    memcpy(this->stream_.get() + offset, &this->timestamp_, sizeof(uint64_t));
    offset += sizeof(uint64_t);

    // 相机信息和帧类型
    uint8_t ck = this->camera_info_.uav_id << 4 | this->camera_info_.camera_type << 2 | this->key_frame_;
    memcpy(this->stream_.get() + offset, &ck, 1);
    offset += sizeof(uint8_t);

    // H矩阵
    memcpy(this->stream_.get() + offset, this->homography_.data, sizeof(Homography));
    offset += sizeof(Homography);

    // 相机外参矩阵
    memcpy(this->stream_.get() + offset, this->camera_info_.cm.data, sizeof(CameraMatrix));
    offset += sizeof(CameraMatrix);

    // 激光距离
    TwoByteFloat laser_distance(this->laser_distance_);
    uint16_t laser_distance_raw = laser_distance.getRawValue();
    memcpy(this->stream_.get() + offset, &laser_distance_raw, 2);
    offset += 2;

    // 目标数量
    memcpy(this->stream_.get() + offset, &this->obj_num_, 1);
    offset += 1;

    // 目标信息
    for (size_t i = 0; i < this->obj_num_; i++)
    {
        pack_objinfo(this->object_info_[i], this->stream_.get() + offset);
        offset += 7;
    }

    //  背景大小
    uint16_t background_len = static_cast<uint16_t>(this->background_len_);
    memcpy(this->stream_.get() + offset, &background_len, 2);
    offset += 2;

    // 背景
    memcpy(stream_.get() + offset, background_.get(), background_len_);
    offset += background_len_;

    // 前景
    memcpy(stream_.get() + offset, foreground_.get(), foreground_len_);
    offset += foreground_len_;
}

void DataPackage::parse_stream(uint8_t *stream, const size_t len)
{
    size_t offset = 0;

    // 时间戳
    memcpy(&this->timestamp_, stream + offset, sizeof(uint64_t));
    offset += sizeof(uint64_t);

    // 相机信息和帧类型
    uint8_t ck;
    memcpy(&ck, stream + offset, 1);
    this->camera_info_.uav_id = ck >> 4;
    this->camera_info_.camera_type = (ck >> 2) & 0x3;
    this->key_frame_ = ck & 0x1;
    offset += sizeof(uint8_t);

    // H矩阵
    memcpy(this->homography_.data, stream + offset, sizeof(Homography));
    offset += sizeof(Homography);

    // 相机外参矩阵
    memcpy(this->camera_info_.cm.data, stream + offset, sizeof(CameraMatrix));
    offset += sizeof(CameraMatrix);

    // 激光距离
    uint16_t laser_distance_raw;
    memcpy(&laser_distance_raw, stream + offset, 2);
    this->laser_distance_ = TwoByteFloat(laser_distance_raw).toFloat();
    offset += 2;

    // 目标数量
    memcpy(&this->obj_num_, stream + offset, 1);
    offset += 1;

    // 目标信息
    for (size_t i = 0; i < this->obj_num_; i++)
    {
        ObjectInfo obj;
        unpack_objinfo(stream + offset, obj);
        this->object_info_.push_back(obj);
        offset += 7;
    }

    //  背景大小
    uint16_t background_len;
    memcpy(&background_len, stream + offset, 2);
    this->background_len_ = background_len;
    offset += 2;
    this->foreground_len_ = len - offset - background_len;
    this->foreground_ = std::make_unique<uint8_t[]>(this->foreground_len_);
    this->background_ = std::make_unique<uint8_t[]>(this->background_len_);

    // 背景
    memcpy(background_.get(), stream + offset, background_len_);
    offset += background_len_;

    // 前景
    memcpy(foreground_.get(), stream + offset, foreground_len_);
    offset += foreground_len_;
}

uint64_t DataPackage::get_timestamp() const
{
    return this->timestamp_;
}

bool DataPackage::is_keyframe() const
{
    return this->key_frame_;
}

uint8_t DataPackage::get_uav_id() const
{
    return this->camera_info_.uav_id;
}

uint8_t DataPackage::get_camera_type() const
{
    return this->camera_info_.camera_type;
}

Homography DataPackage::get_homography() const
{
    return this->homography_;
}

CameraMatrix DataPackage::get_camera_matrix() const
{
    return this->camera_info_.cm;
}

uint8_t DataPackage::get_obj_num() const
{
    return this->obj_num_;
}

std::vector<ObjectInfo> DataPackage::get_object_info() const
{
    return this->object_info_;
}

float DataPackage::get_laser_distance() const
{
    return this->laser_distance_;
}

const uint8_t *DataPackage::get_stream() const
{
    return this->stream_.get();
}

const size_t DataPackage::get_stream_length() const
{
    return this->stream_len_;
}

const uint8_t *DataPackage::get_imgcode() const
{
    return this->imgcode_.get();
}

const size_t DataPackage::get_imgcode_length() const
{
    return this->imgcode_len_;
}

const uint8_t *DataPackage::get_foreground() const
{
    return this->foreground_.get();
}

const size_t DataPackage::get_foreground_length() const
{
    return this->foreground_len_;
}

const uint8_t *DataPackage::get_background() const
{
    return this->background_.get();
}

const size_t DataPackage::get_background_length() const
{
    return this->background_len_;
}
void DataPackage::clear()
{
    this->timestamp_ = 0;
    this->camera_info_.uav_id = 0;
    this->camera_info_.camera_type = 0;
    this->camera_info_.cm = {0};
    this->obj_num_ = 0;
    this->laser_distance_ = 0;
    this->object_info_.clear();
    this->homography_ = {0};
    this->foreground_.reset();
    this->background_.reset();
    this->stream_.reset();
    this->key_frame_ = false;
    this->rgb_.release();
    this->ir_.release();
    this->lowlight_.release();
}

Bbox DataPackage::quant_bbox(const Bbox bbox, const uint32_t align_bits) const
{
    uint32_t align_mask = (1 << align_bits) - 1;
    Bbox quanted_bbox;

    // 将坐标向下对齐到2^align_bits的倍数
    quanted_bbox.x = bbox.x & ~align_mask;
    quanted_bbox.y = bbox.y & ~align_mask;

    // 将宽高向上对齐到2^align_bits的倍数
    quanted_bbox.w = ((bbox.w + align_mask) & ~align_mask);
    quanted_bbox.h = ((bbox.h + align_mask) & ~align_mask);

    return quanted_bbox;
}
// TODO: implement SIMD optimization
Bbox DataPackage::align_bbox(const Bbox bbox, const uint32_t align_bits) const
{
    Bbox aligned_bbox;
    aligned_bbox.x = bbox.x >> align_bits;
    aligned_bbox.y = bbox.y >> align_bits;
    aligned_bbox.w = bbox.w >> align_bits;
    aligned_bbox.h = bbox.h >> align_bits;

    return aligned_bbox;
}

// TODO: implement SIMD optimization
Bbox DataPackage::unalign_bbox(const Bbox aligned_bbox, const uint32_t align_bits) const
{
    Bbox bbox;
    bbox.x = aligned_bbox.x << align_bits;
    bbox.y = aligned_bbox.y << align_bits;
    bbox.w = aligned_bbox.w << align_bits;
    bbox.h = aligned_bbox.h << align_bits;
    return bbox;
}

void DataPackage::pack_objinfo(const ObjectInfo &obj, uint8_t *data)
{
    // 对齐bbox
    auto aligned_rect = this->align_bbox(obj.rect, this->align_bits_);

    // 确保初始化为0
    std::fill_n(data, 7, 0);

    // 使用位运算打包数据
    // Byte 0: [label(2) | tracker_id_high(6)]
    uint16_t tracker_id_high = (obj.tracker_id >> 10) & 0x3F; // 只取需要的6位
    data[0] = static_cast<uint8_t>(
        ((obj.label & 0x03) << 6) | tracker_id_high);

    // Byte 1: [tracker_id_low(8)]
    data[1] = static_cast<uint8_t>(obj.tracker_id & 0xFF);

    // Bytes 2-5: High 8 bits of x, y, w, h
    data[2] = static_cast<uint8_t>(aligned_rect.x >> 2);
    data[3] = static_cast<uint8_t>(aligned_rect.y >> 2);
    data[4] = static_cast<uint8_t>(aligned_rect.w >> 2);
    data[5] = static_cast<uint8_t>(aligned_rect.h >> 2);

    // Byte 6: [x_low(2) | y_low(2) | w_low(2) | h_low(2)]
    data[6] = static_cast<uint8_t>(
        ((aligned_rect.x & 0x3) << 6) |
        ((aligned_rect.y & 0x3) << 4) |
        ((aligned_rect.w & 0x3) << 2) |
        (aligned_rect.h & 0x3));
}

void DataPackage::unpack_objinfo(const uint8_t *data, ObjectInfo &obj)
{
    // 解析label
    obj.label = (data[0] >> 6) & 0x03;

    // 解析tracker_id (16位)
    obj.tracker_id = static_cast<uint16_t>(
        ((data[0] & 0x3F) << 10) | // 高6位
        data[1]                    // 低8位
    );

    // 解析矩形坐标和尺寸
    Bbox aligned_rect;
    aligned_rect.x = (data[2] << 2) | ((data[6] >> 6) & 0x3);
    aligned_rect.y = (data[3] << 2) | ((data[6] >> 4) & 0x3);
    aligned_rect.w = (data[4] << 2) | ((data[6] >> 2) & 0x3);
    aligned_rect.h = (data[5] << 2) | (data[6] & 0x3);
    obj.rect = this->unalign_bbox(aligned_rect, this->align_bits_);
}
