// Copyright Cellink 2022 GPLv3

#pragma once

#include "../../inc/MarlinConfigPre.h"

#include "crc.h"

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
    NOF_CMDS
};

enum class Index : uint8_t {
    All,
    One,
    Two,
    Three
};

struct Packet
{
    Index ph_index;
    Command command;
    const uint8_t* payload;
    uint8_t payload_size;
    uint16_t crc;
    constexpr Packet(Index index, Command cmd, const uint8_t* message_payload, uint8_t message_size)
        : ph_index(index)
        , command(cmd)
        , payload(message_payload)
        , payload_size(message_size)
        , crc(crc16_from_bytes(payload, payload_size))
    {}
    constexpr Packet(): ph_index(Index::All), command(Command::ACK), payload(nullptr), payload_size(0), crc(0) {}
    Packet(Index index, Command cmd, const void* message_payload, uint8_t message_size)
        : ph_index(index)
        , command(cmd)
        , payload(static_cast<const uint8_t*>(message_payload))
        , payload_size(message_size)
        , crc(crc16_from_data(payload, payload_size))
    {}
};

enum class Result {
    Ok,
    Busy,
    TooShort,
    BadSize,
    ChecksumFailed
};

Result send(const Packet& request, HardwareSerial& serial)
{
    static_assert(sizeof(Index) == 1 && sizeof(Command) == 2, "Unexpected packet enum byte-widths");
    uint8_t index_byte;
    memcpy(&index_byte, &request.ph_index, 1);
    uint8_t command_bytes[2];
    memcpy(command_bytes, &request.command, 2);
    uint8_t crc_bytes[2];
    memcpy(crc_bytes, &request.crc, 2);

    serial.write(index_byte);
    serial.write(command_bytes, 2);
    serial.write(request.payload_size);
    serial.write(request.payload, request.payload_size);
    serial.write(crc_bytes, 2);

    // should read an ACK after this write to assure complete transaction

    return Result::Ok;
}

struct Response {
    Packet packet;
    Result result;
};

Response receive(HardwareSerial& serial)
{
    // this seems bug-prone...
    static uint8_t packet_buffer[64];

    Packet incoming{};

    auto bytes_received = serial.readBytes(packet_buffer, 64);

    if (bytes_received < 6)
        return Response{incoming, Result::TooShort};
    memcpy(&incoming.ph_index, &packet_buffer[0], 1);
    memcpy(&incoming.command, &packet_buffer[1], 2);
    memcpy(&incoming.payload_size, &packet_buffer[3], 1);
    if (incoming.payload_size !=  bytes_received - 6)
        return Response{incoming, Result::BadSize};
    incoming.payload = &packet_buffer[4];
    memcpy(&incoming.crc, &packet_buffer[4 + incoming.payload_size], 2);
    uint16_t crc = crc16_from_bytes(incoming.payload, incoming.payload_size);

    if (crc != incoming.crc)
        return Response{incoming, Result::ChecksumFailed};

    return Response{incoming, Result::Ok};

    // ACK would go here
}

class Controller
{
    HardwareSerial& bus;
    Index index;

public:
    Controller(HardwareSerial& ph_bus, Index ph_index)
        : bus(ph_bus)
        , index(ph_index)
    {}
    Result set_temp(float temperature)
    {
        static constexpr size_t PAYLOAD_SIZE = sizeof(uint16_t);
        uint16_t converted_temp = static_cast<uint16_t>(temperature * 100.0f + 30000.0f);
        uint8_t payload[PAYLOAD_SIZE]{};
        memcpy(payload, &converted_temp, PAYLOAD_SIZE);
        Packet request(index, Command::SET_TEMP, payload, PAYLOAD_SIZE);
        return send(request, bus);
    }
};

} // namespace printhead