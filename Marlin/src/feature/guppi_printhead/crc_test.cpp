#include "packet.h"

using namespace printhead;

static constexpr auto crc_correctness_test = []{
    static constexpr uint8_t test_str[] = u8"extrude 20 units";
    static constexpr uint16_t value = crc16_from_bytes(test_str, 16);
    static_assert(value == 21729, "CRC calculation is incorrect");
};

static constexpr auto crc_correctness_from_bytes_test = []{
    static constexpr std::array<uint8_t, 17> test_arr{u8"extrude 20 units"};
    static constexpr uint16_t crc = calculate_crc16(test_arr);
    static_assert(crc == 21729, "CRC from bytes is incorrect");
};