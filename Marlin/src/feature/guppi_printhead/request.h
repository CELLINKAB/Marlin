// Copyright Cellink 2022 GPLv3

#pragma once

#include "../../inc/MarlinConfigPre.h"

#include "crc.h"

#include <array>

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

enum class Index : uint16_t {
    One,
    Two,
    Three,
    None = 0xAA,
    All = 0xff, // max value for Marlin tool
};

struct Packet
{
    Index ph_index;
    Command command;
    uint16_t payload_size;
    const uint8_t* payload;
    uint16_t crc;
    constexpr Packet(Index index, Command cmd, const uint8_t* message_payload, uint8_t message_size)
        : ph_index(index)
        , command(cmd)
        , payload_size(message_size)
        , payload(message_payload)
        , crc(crc16_from_bytes(payload, payload_size))
    {}
    constexpr Packet()
        : ph_index(Index::None)
        , command(Command::ACK)
        , payload_size(0)
        , payload(nullptr)
        , crc(0)
    {}
    constexpr Packet(Index index, Command cmd)
        : ph_index(index)
        , command(cmd)
        , payload_size(0)
        , payload(nullptr)
        , crc(0)
    {}
    Packet(Index index, Command cmd, const void* message_payload, uint8_t message_size)
        : ph_index(index)
        , command(cmd)
        , payload_size(message_size)
        , payload(static_cast<const uint8_t*>(message_payload))
        , crc(crc16_from_data(payload, payload_size))
    {}

    using HeaderBytes = std::array<uint8_t, 6>;
    using CrcBytes = std::array<uint8_t, 2>;

    HeaderBytes header_bytes() const;

    CrcBytes crc_bytes() const;
};

enum class Result {
    OK,
    BAD_CRC,
    BUSY,
    PACKET_TOO_SHORT,
    BAD_PAYLOAD_SIZE,
};

enum class ErrorPayload {
    OK,
    INVALID_CRC,
    INVALID_CMD,
    WDOG_RESET,
    MAX_TEMP,
    MIN_TEMP,
    ADDRESS_SETUP,
    CHIP_SELECT,
    MICRO_STEP,
    MOTOR_DIR,
    HEATER_TIMEOUT,
    HEATER_SENSOR_ERROR,
    HEATER_OVERTEMP,
    ENDSTOP_TOP_HIT,
    ENDSTOP_BOTTOM_HIT,
    MOTOR_TIMEOUT,
    HEATER_BROKEN,
    PHYSICS,
    HEATER_DISCONNECTED,
    INVALID_PARAM,
    VALVE_BROKEN,
    PARAM_STORE,
    PARAM_LOAD,
};

Result send(const Packet& request, HardwareSerial& serial);

struct Response
{
    Packet packet;
    Result result;
};

Response receive(HardwareSerial& serial);

Response send_and_receive(Packet packet, HardwareSerial& serial);

enum class ExtruderDirection : uint8_t {
    Extrude,
    Retract
};

class Controller
{
    HardwareSerial& bus;

public:
    Controller(HardwareSerial& ph_bus)
        : bus(ph_bus)
    {}
    void init();

    // Metadata methods
    Response get_info(Index index);
    Response get_fw_version(Index index);
    // Temperature methods
    Result set_temperature(Index index, float temperature);
    Response get_temperature(Index index);
    Result set_pid(Index index, float p, float i, float d);
    Response get_pid(Index index);
    // Extruder Stepper driver methods
    Result set_extrusion_speed(Index index, feedRate_t feedrate);
    Response get_extrusion_speed(Index index);
    Result set_extruder_stallguard_threshold(Index index, uint8_t threshold);
    Response get_extruder_stallguard_threshold(Index index);
    Result set_extruder_microsteps(Index index, uint8_t microsteps);
    Response get_extruder_microsteps(Index index);
    Result set_extruder_rms_current(Index index, uint16_t mA);
    Response get_extruder_rms_current(Index index);
    Result set_extruder_hold_current(Index index, uint16_t mA);
    Response get_extruder_hold_current(Index index);
    Result home_extruder(Index index, ExtruderDirection direction);
    Result start_extruding(Index index);
    Result stop_extruding(Index index);
    // Slider Valve driver methods
    Result set_valve_speed(Index index, feedRate_t feedrate);
    Response get_valve_speed(Index index);
    Result set_valve_stallguard_threshold(Index index, uint8_t threshold);
    Response get_valve_stallguard_threshold(Index index);
    Result set_valve_microsteps(Index index, uint8_t microsteps);
    Response get_valve_microsteps(Index index);
    Result set_valve_rms_current(Index index, uint16_t mA);
    Response get_valve_rms_current(Index index);
    Result set_valve_hold_current(Index index, uint16_t mA);
    Result home_slider_valve(Index index);
    Result move_slider_valve(Index index, uint16_t steps);


};

} // namespace printhead