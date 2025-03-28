#include "utils/sendFrament.h"

void sendFragmented(std::shared_ptr<UDPOperation> server, std::vector<uint8_t>& data, uint32_t magic) {
    constexpr size_t FRAG_SIZE = 1400; // 留出72字节给头部和其他元数据
    
    PacketHeader header;
    header.magic = magic;
    header.total_frags = (data.size() + FRAG_SIZE - 1) / FRAG_SIZE;
    header.data_size = data.size();

    for(uint16_t i=0; i<header.total_frags; ++i) {
        // 构造分片数据包
        std::vector<uint8_t> packet(sizeof(PacketHeader));
        header.frag_num = i;
        
        // 添加头部
        memcpy(packet.data(), &header, sizeof(header));
        
        // 添加数据分片
        auto start = data.begin() + i*FRAG_SIZE;
        auto end = (i+1 == header.total_frags) ? data.end() : start + FRAG_SIZE;
        packet.insert(packet.end(), start, end);

        // 发送分片
        if(server->send_buffer(reinterpret_cast<char*>(packet.data()), packet.size())) {
            // perror(("sendto fragment " + std::to_string(i)).c_str());
        }
    }
}