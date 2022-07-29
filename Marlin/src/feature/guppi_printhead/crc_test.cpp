#include "crc.h"


static constexpr auto crc_correctness_test = []{
    static constexpr uint8_t test_str[] = u8"extrude 20 units";
    static constexpr uint16_t value = crc16_from_bytes(test_str, 16);
    static_assert(value == 21729, "CRC calculation is incorrect");
};