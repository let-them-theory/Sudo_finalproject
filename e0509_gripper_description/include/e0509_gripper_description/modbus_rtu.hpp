#pragma once

#include <cstdint>
#include <vector>
#include <algorithm>

namespace modbus_rtu {

constexpr uint8_t SLAVE_ID = 1;
constexpr uint16_t REG_TORQUE_ENABLE = 256;
constexpr uint16_t REG_PRESENT_POSITION = 276;
constexpr uint16_t REG_GOAL_POSITION = 282;

inline uint16_t crc16(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (auto byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

inline std::vector<uint8_t> make_frame(const std::vector<uint8_t>& data) {
    auto frame = data;
    uint16_t crc = crc16(data);
    frame.push_back(crc & 0xFF);
    frame.push_back((crc >> 8) & 0xFF);
    return frame;
}

inline std::vector<uint8_t> fc06_torque_enable() {
    return make_frame({SLAVE_ID, 0x06, 0x01, 0x00, 0x00, 0x01});
}

inline std::vector<uint8_t> fc16_position(int position) {
    position = std::clamp(position, 0, 700);
    return make_frame({
        SLAVE_ID, 0x10,
        0x01, 0x1A,
        0x00, 0x02,
        0x04,
        static_cast<uint8_t>((position >> 8) & 0xFF),
        static_cast<uint8_t>(position & 0xFF),
        0x00, 0x00
    });
}

// FC03: Read Holding Registers - read present position (register 276, 2 registers)
inline std::vector<uint8_t> fc03_read_present_position() {
    return make_frame({
        SLAVE_ID, 0x03,
        static_cast<uint8_t>((REG_PRESENT_POSITION >> 8) & 0xFF),
        static_cast<uint8_t>(REG_PRESENT_POSITION & 0xFF),
        0x00, 0x02  // 2 registers
    });
}

// Parse FC03 response to extract position value
// Response format: [slave_id, 0x03, byte_count, data_hi, data_lo, ..., crc_lo, crc_hi]
inline int parse_present_position(const std::vector<uint8_t>& response) {
    // Minimum response: slave(1) + fc(1) + byte_count(1) + data(4) + crc(2) = 9 bytes
    if (response.size() < 9) return -1;
    if (response[1] != 0x03) return -1;  // Not FC03 response

    uint8_t byte_count = response[2];
    if (byte_count < 4) return -1;

    // Present position is in the first 2 data bytes (big-endian)
    int position = (response[3] << 8) | response[4];
    return std::clamp(position, 0, 700);
}

}  // namespace modbus_rtu
