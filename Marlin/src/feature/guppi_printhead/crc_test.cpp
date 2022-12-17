#include "packet.h"

using namespace printhead;

namespace test {

template<typename T, size_t N, size_t M>
constexpr auto compare_arrays(std::array<T,N> a, std::array<T,M> b) -> bool
{
    bool no_mismatches = true;
    for (size_t i = 0; i < b.size(); ++i)
        no_mismatches = no_mismatches && (a[i] == b[i]);
    return no_mismatches;
};

void cat_cats()
{
    static constexpr std::array full_arr{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    static constexpr std::array part_1{0, 1, 2, 3};
    static constexpr std::array part_2{4, 5, 6, 7, 8};
    static constexpr std::array part_3{9};
    static constexpr std::array reconstructed = concatenate(part_1, part_2, part_3);
    static_assert(sizeof(full_arr) == sizeof(reconstructed) && full_arr.size() == reconstructed.size(),
                  "concatenation yielded length mismatch");
    static_assert(compare_arrays(full_arr, reconstructed), "concatenation yielded value mismatch");
}

void byte_array_conversion()
{
    static constexpr std::array<uint8_t, 4> expected{0x04, 0x03, 0x02, 0x01};
    static constexpr auto from_conversion = byte_array<uint32_t>(0x01020304);
    static_assert(from_conversion.size() == 4, "conversion from integral to byte array yielded bad length");
    static_assert(compare_arrays(expected, from_conversion), "bad conversion from integral to bytes");
}

void crc_correctness()
{
    static constexpr uint8_t test_str[] = u8"extrude 20 units";
    static constexpr uint16_t value = crc16_from_bytes(test_str, 16);
    static_assert(value == 21729, "CRC calculation is incorrect");
}

void crc_correctness_from_bytes_matches_raw()
{
    static constexpr std::array<uint8_t, 17> test_arr{u8"extrude 20 units"};
    static constexpr uint16_t crc_from_array = calculate_crc16(test_arr);
    static constexpr uint16_t crc_from_data = crc16_from_bytes(test_arr.data(), test_arr.size());
    static_assert(crc_from_array == crc_from_data, "CRC from bytes is incorrect");
}

void void_packet_bytes()
{
    static constexpr std::array<uint8_t, 8> known_good_arr{0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x3E, 0x83};
    static constexpr Packet packet(Index::One, Command::GET_MEASURED_TEMP);
    static_assert(known_good_arr.size() == packet.bytes().size(), "packet gives bad size arrays");
    // static_assert(compare_arrays(known_good_arr, packet.header_bytes()), "packet gives bad bytes in the header");
    // static_assert(compare_arrays(known_good_arr, packet.bytes()), "packet gives bad bytes in the CRC");
}

} // namespace test