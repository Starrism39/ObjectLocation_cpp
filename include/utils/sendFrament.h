#include <iostream>
#include <vector>
#include "utils/udp_operation.h"
struct PacketHeader {
    uint32_t magic = 0xAA55CC33;
    uint16_t total_frags;
    uint16_t frag_num;
    uint32_t data_size;
};
void sendFragmented(std::shared_ptr<UDPOperation> server, std::vector<uint8_t>& data, uint32_t magic);

