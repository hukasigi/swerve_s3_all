#pragma once

#include "constants.hpp"

std::vector<uint8_t> positionToPayload(const x_y_theta_deg& position) {
    const int16_t values[3] = {
        static_cast<int16_t>(position.x),
        static_cast<int16_t>(position.y),
        static_cast<int16_t>(position.deg),
    };

    std::vector<uint8_t> payload(6);

    for (int i = 0; i < 3; ++i) {
        const uint16_t value = static_cast<uint16_t>(values[i]);
        payload[i * 2]       = static_cast<uint8_t>(value & 0xFF);
        payload[i * 2 + 1]   = static_cast<uint8_t>((value >> 8) & 0xFF);
    }

    return payload;
}

int16_t readInt16(const std::vector<uint8_t>& data, size_t index) {
    uint16_t value = static_cast<uint16_t>(data[index]) | (static_cast<uint16_t>(data[index + 1]) << 8);

    return static_cast<int16_t>(value);
}