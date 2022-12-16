/*
 * crc.h
 *
 *	Created on: 22 aug. 2016
 *	Author: Sterna
 *	The contents of this file is for internal use at CELLINK only.
 *	You are not allowed to distribute the contents of this file without prior written approval from CELLINK AB.
 *	If you have received this file in error, you are kindly asked to
 *	notify us of the reception and delete the file.
 *	Copyright (c) CELLINK AB
 */

#ifndef CRC_H_
#define CRC_H_

#include <array>
#include <cstdint>
#include <cstdio>
#include <tuple>
#include <type_traits>

namespace printhead {
enum class Command : uint16_t {
    SET_START = 0, /* Should not be used */
    SET_PRINTER_HEAD = 1,
    GET_PRINTER_HEAD_TYPE = 2,
    SET_EXTRUSION_SPEED = 3,
    GET_EXTRUSION_SPEED = 4,
    SET_TEMP = 5,
    GET_SET_TEMP = 6,
    SET_RGB_LED = 7,
    GET_RGB_LED = 8,
    ERROR = 9,
    HEARTBEAT = 10,
    GET_SYSTICK = 11,
    ENABLE_ADDRESS_SETUP = 12,
    DISABLE_ADDRESS_SETUP = 13,
    SET_PRINTHEAD_ADDRESS = 14,
    SET_SOFT_MIN_TEMP = 15,
    GET_SOFT_MIN_TEMP = 16,
    SET_SOFT_MAX_TEMP = 17,
    GET_SOFT_MAX_TEMP = 18,
    GET_UNIQUE_ID = 19,
    SET_RETRACT_VOLUME = 20,
    GET_RETRACT_VOLUME = 21,
    SET_STEP_VOLUME = 22,
    GET_STEP_VOLUME = 23,
    SET_EXTRUSION_DIRECTION = 24,
    GET_EXTRUSION_DIRECTION = 25,
    GET_ENDSTOP_STATUS = 26,
    MOVE_TO_HOME_POSITION = 27,
    SET_MICROSTEP = 28,
    GET_MICROSTEP = 29,
    SET_MOTOR_STATE = 30,
    GET_MOTOR_STATE = 31,
    GET_MEASURED_TEMP = 32,
    GET_RAW_TEMP = 33,
    GET_ALL = 34,
    GET_DEVICE_INFO = 35,
    SET_PID = 36,
    GET_PID = 37,
    SET_TEMP_FILTER = 38,
    GET_TEMP_FILTER = 39,
    SET_HEATER_MAX = 40,
    GET_HEATER_MAX = 41,
    DEBUG_SET_HEATER = 42,
    DEBUG_SET_MOTOR_CURRENT = 43,
    DEBUG_GET_ALL = 44,
    DEBUG_GET_ALL_HEATER = 45,
    ACK = 46,
    SET_ROOM_TEMP = 47,
    GET_ROOM_TEMP = 48,
    SET_FAN_SPEED = 49,
    GET_FAN_SPEED = 50,
    GET_STATUS = 51,
    SET_HEATER_SELF_TEST_START = 52,
    GET_HEATER_SELF_TEST_RESULT = 53,
    DISABLE_TEMP_CONTROL = 54,
    GET_MCU_INTERNAL_TEMP = 55,
    GET_TEMP_CONTROL_CURRENT = 56,
    SET_EXTERNAL_PWM = 57,
    GET_EXTERNAL_PWM = 58,
    SET_AMP_CORRECTIONS = 59,
    GET_AMP_CORRECTIONS = 60,
    STORE_ALL_PARAMS = 61,
    RESET_PARAMS_TO_DEFAULT = 62,
    RESET_PARAM_FLASH = 63,
    RESTORE_SAVED_PARAMS = 64,
    SET_NOZZLE_DISTANCE = 65,
    GET_SET_NOZZLE_DISTANCE = 66,
    SET_TEMP_CORR_MODEL = 67,
    GET_TEMP_CORR_MODEL = 68,
    SET_TEMP_I_PARAMS = 69,
    GET_TEMP_I_PARAMS = 70,
    SET_TEMP_REG_OVERSHOOT = 71,
    GET_TEMP_REG_OVERSHOOT = 72,
    SET_TEMP_CORR_LUT_POINTS = 73,
    GET_TEMP_CORR_LUT_POINTS = 74,
    CLEAR_TEMP_CORR_LUT_POINTS = 75,
    SET_TEMP_CORR_STATE = 76,
    SET_HEATER_SELF_TEST_PARAMS = 77,
    GET_HEATER_SELF_TEST_PARAMS = 78,
    SET_EXTERNAL_CHIP_PARAMS = 84,
    GET_SW_VERSION = 99,
    PNEUMATIC_START = 100,
    SYRINGEPUMP_START = 200,
    SYRINGEPUMP_SET_ESTOP_THRESH = 201,
    SYRINGEPUMP_GET_ESTOP_THRESH = 202,
    SYRINGEPUMP_GET_ESTOP_VALS = 203,
    SYRINGEPUMP_SET_FULLSTEP_VOLUME = 204,
    SYRINGEPUMP_GET_FULLSTEP_VOLUME = 205,
    SYRINGEPUMP_DEBUG_ADD_STEPS = 206,
    PNEUMATIC250C_START = 300,
    PNEUMATIC250C_SET_AMP_CORRECTIONS = 301,
    PNEUMATIC250C_GET_AMP_CORRECTIONS = 302,
    PNEUMATIC_COOLED_START = 400,
    PNEUMATIC_COOLED_SET_POLARITY = 401,
    PNEUMATIC_COOLED_START_BUCK_CALIBRATION = 402,
    PNEUMATIC_COOLED_GET_BUCK_CALIBRATION = 403,
    PNEUMATIC_COOLED_GET_BUCK_MEASURED_VOLTAGE = 404,
    PNEUMATIC_COOLED_SET_MAX_COOLING_VOLTAGE = 405,
    PNEUMATIC_COOLED_GET_MAX_COOLING_VOLTAGE = 406,
    PNEUMATIC_COOLED_SET_AMP_CORRECTIONS = 407,
    PNEUMATIC_COOLED_GET_AMP_CORRECTIONS = 408,
    CAMERA_HD_START = 500,
    INKJET_START = 600,
    INKJET_SET_PEAKTIME = 601,
    INKJET_GET_PEAKTIME = 602,
    INKJET_SET_OPENTIME = 603,
    INKJET_GET_OPENTIME = 604,
    INKJET_SET_CYCLETIME = 605,
    INKJET_GET_CYCLETIME = 606,
    INKJET_SET_PEAKCURRENT = 607,
    INKJET_GET_PEAKCURRENT = 608,
    INKJET_SET_HOLDCURRENT = 609,
    INKJET_GET_HOLDCURRENT = 610,
    INKJET_SET_AIR_SUPPLY = 611,
    INKJET_VALVE_SELF_TEST_START = 612,
    INKJET_VALVE_SELF_TEST_GET_RESULT = 613,
    SYRINGEPUMP_20ML_START = 700,
    CURING_START = 800,
    CURING_SET_OUTPUT_POWER = 801,
    CURING_GET_SET_OUTPUT_POWER = 802,
    CURING_SET_OUTPUT_MODE = 803,
    CURING_GET_WAVELENGTH_ID = 804,
    CURING_GET_PHOTO_FEEDBACK = 805,
    CURING_GET_PHOTO_FEEDBACK_DEBUG = 806,
    CURING_START_LED_SELFTEST = 807,
    CURING_GET_LED_SELFTEST_RESULT = 808,
    CURING_GET_LED_VOLTAGE = 809,
    TYPE_PNEUMATIC_10ML_START = 900,
    SLIDER_MOVE_TO_HOME_POSITION = 1001,
    DEBUG_ADD_SLIDER_STEPS = 1006,
    DEBUG_SET_FAN_PWM = 1007,
    DEBUG_SET_TEM_PWM = 1008,
    DEBUG_GET_FAN_PWM = 1009,
    DEBUG_GET_TEM_PWM = 1010,
    DEBUG_GET_TEMPERATURE = 1011,

    NOF_CMDS
};

enum class Index : uint16_t {
    One,
    Two,
    Three,
    None = 0xAA,
    All = 0xff, // max value for Marlin tool
};

//The first data for the crc
static constexpr uint16_t CRC_INIT_BYTE16 = 0xCAFE;

static constexpr uint16_t crcTable16[256]
    = {0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A,
       0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7, 0x0919, 0x1890, 0x2A0B, 0x3B82, 0x4F3D, 0x5EB4,
       0x6C2F, 0x7DA6, 0x8551, 0x94D8, 0xA643, 0xB7CA, 0xC375, 0xD2FC, 0xE067, 0xF1EE, 0x1232,
       0x03BB, 0x3120, 0x20A9, 0x5416, 0x459F, 0x7704, 0x668D, 0x9E7A, 0x8FF3, 0xBD68, 0xACE1,
       0xD85E, 0xC9D7, 0xFB4C, 0xEAC5, 0x1B2B, 0x0AA2, 0x3839, 0x29B0, 0x5D0F, 0x4C86, 0x7E1D,
       0x6F94, 0x9763, 0x86EA, 0xB471, 0xA5F8, 0xD147, 0xC0CE, 0xF255, 0xE3DC, 0x2464, 0x35ED,
       0x0776, 0x16FF, 0x6240, 0x73C9, 0x4152, 0x50DB, 0xA82C, 0xB9A5, 0x8B3E, 0x9AB7, 0xEE08,
       0xFF81, 0xCD1A, 0xDC93, 0x2D7D, 0x3CF4, 0x0E6F, 0x1FE6, 0x6B59, 0x7AD0, 0x484B, 0x59C2,
       0xA135, 0xB0BC, 0x8227, 0x93AE, 0xE711, 0xF698, 0xC403, 0xD58A, 0x3656, 0x27DF, 0x1544,
       0x04CD, 0x7072, 0x61FB, 0x5360, 0x42E9, 0xBA1E, 0xAB97, 0x990C, 0x8885, 0xFC3A, 0xEDB3,
       0xDF28, 0xCEA1, 0x3F4F, 0x2EC6, 0x1C5D, 0x0DD4, 0x796B, 0x68E2, 0x5A79, 0x4BF0, 0xB307,
       0xA28E, 0x9015, 0x819C, 0xF523, 0xE4AA, 0xD631, 0xC7B8, 0x48C8, 0x5941, 0x6BDA, 0x7A53,
       0x0EEC, 0x1F65, 0x2DFE, 0x3C77, 0xC480, 0xD509, 0xE792, 0xF61B, 0x82A4, 0x932D, 0xA1B6,
       0xB03F, 0x41D1, 0x5058, 0x62C3, 0x734A, 0x07F5, 0x167C, 0x24E7, 0x356E, 0xCD99, 0xDC10,
       0xEE8B, 0xFF02, 0x8BBD, 0x9A34, 0xA8AF, 0xB926, 0x5AFA, 0x4B73, 0x79E8, 0x6861, 0x1CDE,
       0x0D57, 0x3FCC, 0x2E45, 0xD6B2, 0xC73B, 0xF5A0, 0xE429, 0x9096, 0x811F, 0xB384, 0xA20D,
       0x53E3, 0x426A, 0x70F1, 0x6178, 0x15C7, 0x044E, 0x36D5, 0x275C, 0xDFAB, 0xCE22, 0xFCB9,
       0xED30, 0x998F, 0x8806, 0xBA9D, 0xAB14, 0x6CAC, 0x7D25, 0x4FBE, 0x5E37, 0x2A88, 0x3B01,
       0x099A, 0x1813, 0xE0E4, 0xF16D, 0xC3F6, 0xD27F, 0xA6C0, 0xB749, 0x85D2, 0x945B, 0x65B5,
       0x743C, 0x46A7, 0x572E, 0x2391, 0x3218, 0x0083, 0x110A, 0xE9FD, 0xF874, 0xCAEF, 0xDB66,
       0xAFD9, 0xBE50, 0x8CCB, 0x9D42, 0x7E9E, 0x6F17, 0x5D8C, 0x4C05, 0x38BA, 0x2933, 0x1BA8,
       0x0A21, 0xF2D6, 0xE35F, 0xD1C4, 0xC04D, 0xB4F2, 0xA57B, 0x97E0, 0x8669, 0x7787, 0x660E,
       0x5495, 0x451C, 0x31A3, 0x202A, 0x12B1, 0x0338, 0xFBCF, 0xEA46, 0xD8DD, 0xC954, 0xBDEB,
       0xAC62, 0x9EF9, 0x8F70};

/**
 * @brief Single step in a CRC16 calculation
 * 
 * @param crc_state CRC output from this function previously, or initial value
 * @param next_byte 
 * @return constexpr uint16_t updated CRC value
 */
[[nodiscard]] inline constexpr uint16_t crc16_iteration(uint16_t crc_state,
                                                        const uint8_t next_byte) noexcept
{
    const uint16_t magic = crcTable16[((crc_state >> 8) ^ next_byte)];
    crc_state = (crc_state << 8) ^ magic;
    return crc_state;
}

template<typename Type, std::size_t... sizes>
constexpr auto concatenate(const std::array<Type, sizes>&... arrays)
{
    return std::apply([](auto... elems) -> std::array<Type, (sizes + ...)> { return {{elems...}}; },
                      std::tuple_cat(std::tuple_cat(arrays)...));
}

template<typename T, class = std::enable_if<std::is_integral_v<T>>>
[[nodiscard]] constexpr auto byte_array(T t) noexcept -> std::array<uint8_t, sizeof(T)>
{
    if constexpr (sizeof(T) == 1) return std::array{static_cast<uint8_t>(t)};
    else {
    std::array<uint8_t, sizeof(T)> a{};
    size_t i = 0;
    for (size_t i = 0; i < sizeof(t); ++i) {
        a[i] = (t >> (i++ * sizeof(uint8_t))) & 0xff;
    }
    return a;
    }
}

template<typename T, size_t N>
[[nodiscard]] constexpr auto byte_array(std::array<T, N> t) noexcept
    -> std::array<uint8_t, sizeof(T) * N>
{
    std::array<uint8_t, sizeof(T) * N> a{};
    for (size_t i = 0; i < sizeof(t); ++i) {
        size_t byte_number = i % sizeof(T);
        size_t arr_element = i / sizeof(T);
        a[i] = (t[arr_element] >> (byte_number * sizeof(uint8_t))) & 0xff;
    }
    return a;
}

template<size_t N>
[[nodiscard]] constexpr auto calculate_crc16(const std::array<uint8_t, N>& packet,
                                             uint16_t init_data = CRC_INIT_BYTE16) noexcept
    -> uint16_t
{
    for (const uint8_t b : packet)
        init_data = crc16_iteration(init_data, b);
    return init_data;
}

struct EmptyPacket
{
    Index ph_index;
    Command command;
    uint16_t payload_size;
    uint16_t crc;

    [[nodiscard]] constexpr auto header_bytes() const noexcept
    {
        return concatenate(byte_array(static_cast<uint16_t>(ph_index)),
                           byte_array(static_cast<uint16_t>(command)),
                           byte_array(payload_size));
    }

    EmptyPacket() = default;
    constexpr EmptyPacket(Index index, Command _command)
        : ph_index(index)
        , command(_command)
        , payload_size(0)
        , crc(calculate_crc16(header_bytes()))
    {}

    [[nodiscard]] auto bytes() const noexcept
    {
        return concatenate(header_bytes(), byte_array(crc));
    }

protected:
    constexpr EmptyPacket(Index index, Command _command, uint16_t size, uint16_t payload_crc)
        : ph_index(index)
        , command(_command)
        , payload_size(size)
        , crc(crc)
    {}
};

template<typename PAYLOAD = void>
struct Packet final : public EmptyPacket
{
    PAYLOAD payload;

    [[nodiscard]] constexpr auto payload_bytes() const noexcept { return byte_array(payload); }

    Packet() = default;
    Packet(Index, Command) = delete;
    constexpr Packet(Index index, Command cmd, PAYLOAD message_payload)
        : EmptyPacket(index,
                      cmd,
                      sizeof(message_payload),
                      (calculate_crc16(concatenate(header_bytes(), payload_bytes()))))
        , payload(message_payload)
    {}

    //Packet(Index index, Command cmd, const void* message_payload, uint8_t message_size)
    //    : ph_index(index)
    //    , command(cmd)
    //    , payload_size(message_size)
    //    , payload(static_cast<const uint8_t*>(message_payload))
    //    , crc(crc16_from_data(payload, payload_size))
    //{}

    //using PacketBytes = std::array<uint8_t, (sizeof(ph_index) + sizeof(command) + sizeof(payload_size) + sizeof(payload) + sizeof(crc))>;

    [[nodiscard]] constexpr auto bytes() const noexcept
    {
        return concatenate(header_bytes(), payload_bytes(), byte_array(crc));
    }

};

template<>
struct Packet<void> final : public EmptyPacket
{
    Packet() = default;
    constexpr Packet(Index index, Command cmd)
        : EmptyPacket(index, cmd)
    {}
};

[[nodiscard]] constexpr uint16_t crc16_from_bytes(const uint8_t* data,
                                    size_t len,
                                    uint16_t init_data = CRC_INIT_BYTE16)
{
    uint16_t crc = init_data;

    while (len--) {
        crc = crc16_iteration(crc, *data++);
    }

    return crc;
}

/**
 * @brief Calculate the CRC of a provided memory region, optionally extending from a previous calculation
 * 
 * @param c_ptr data to be checksummed
 * @param len number of bytes to read
 * @param initData if extending from a previous CRC calculation, use the CRC previously calculated here, otherwise the default init value is used
 * @return constexpr uint16_t 
 */
constexpr uint16_t crc16_from_data(const void* c_ptr, size_t len, uint16_t initData = CRC_INIT_BYTE16)
{
    const uint8_t* c = static_cast<const uint8_t*>(c_ptr);
    return crc16_from_bytes(c, len, initData);
}

} // namespace printhead

#endif /* CRC_H_ */
