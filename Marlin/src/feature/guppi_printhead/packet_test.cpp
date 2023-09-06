/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 Cellink [https://github.com/CELLINKAB/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(CHANTARELLE_SUPPORT)

#include "packet.h"
#include "test.h"

using namespace printhead;

namespace test {

void cat_cats()
{
    static constexpr std::array full_arr{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    static constexpr std::array part_1{0, 1, 2, 3};
    static constexpr std::array part_2{4, 5, 6, 7, 8};
    static constexpr std::array part_3{9};
    static constexpr std::array reconstructed = concatenate(part_1, part_2, part_3);
    static_assert_eq(full_arr.size(), reconstructed.size());
    static_assert(array_compare(full_arr, reconstructed));
}

void byte_array_conversion()
{
    static constexpr std::array<uint8_t, 4> expected{0x04, 0x03, 0x02, 0x01};
    static constexpr auto from_conversion = byte_array<uint32_t>(0x01020304);
    static_assert_eq(from_conversion.size(), 4);
    static_assert(array_compare(expected, from_conversion));
}

void crc_correctness()
{
    static constexpr uint8_t test_str[] = u8"extrude 20 units";
    static constexpr uint16_t value = crc16_from_bytes(test_str, 17);
    static_assert_eq(value, 35417);
}

void crc_correctness_from_bytes_matches_raw()
{
    static constexpr std::array<uint8_t, 17> test_arr{u8"extrude 20 units"};
    static constexpr uint16_t crc_from_array = calculate_crc16(test_arr);
    static constexpr uint16_t crc_from_data = crc16_from_bytes(test_arr.data(), test_arr.size());
    static_assert_eq(crc_from_array, crc_from_data);
    static_assert_eq(crc_from_array, 35417);
}

void void_packet_bytes()
{
    static constexpr std::array<uint8_t, 8> known_good_arr{0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0xF4, 0x78};
    static constexpr Packet packet(Index::Two, Command::GET_MEASURED_TEMP);
    static_assert_eq(known_good_arr.size(), packet.bytes().size());
    static_assert(array_compare(known_good_arr, packet.header_bytes()), "packet header doesn't match");
    static_assert(array_compare(known_good_arr, packet.bytes()), "packet CRC doesn't match");
}

} // namespace test

#endif // CHANTARELLE_SUPPORT