#include "utils/protocol.h"

namespace ProtocolUtils {

    // OpenCV矩阵序列化
    void serializeMat(std::vector<uint8_t>& buffer, const cv::Mat& img) {
        serialize<uint32_t>(buffer, img.rows);
        serialize<uint32_t>(buffer, img.cols);
        serialize<int32_t>(buffer, img.type());
        
        if(img.isContinuous()) {
            buffer.insert(buffer.end(), img.data, img.data + img.total() * img.elemSize());
        } else {
            cv::Mat continuousImg = img.clone();
            buffer.insert(buffer.end(), continuousImg.data, 
                         continuousImg.data + continuousImg.total() * continuousImg.elemSize());
        }
    }
    
    // OpenCV矩阵反序列化
    cv::Mat deserializeMat(const uint8_t*& data) {
        uint32_t rows = deserialize<uint32_t>(data);
        uint32_t cols = deserialize<uint32_t>(data);
        int type = deserialize<int32_t>(data);
        // std::cout << "rows: " << rows << " cols: " << cols << " type: " << type << std::endl;
        
        cv::Mat img(rows, cols, type);
        size_t dataSize = rows * cols * img.elemSize();
        memcpy(img.data, data, dataSize);
        data += dataSize;
        return img;
    }
    
    } // namespace ProtocolUtils