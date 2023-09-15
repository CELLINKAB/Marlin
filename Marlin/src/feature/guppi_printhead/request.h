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
#pragma once

#include "../../inc/MarlinConfig.h"

#include <array>

#include "packet.h"

//#define AUTO_REPORT_CHANTARELLE
#if ENABLED(AUTO_REPORT_CHANTARELLE)
#    include "../../libs/autoreport.h"
#endif

namespace printhead {

constexpr double EXTRUSION_RADIUS = DEFAULT_NOMINAL_FILAMENT_DIA / 2;
constexpr double UL_PER_MM = EXTRUSION_RADIUS * EXTRUSION_RADIUS * PI;
constexpr double THREAD_PITCH_MM = 0.6;
constexpr double STEPS_PER_REV = 400;
constexpr double MM_PER_FULL_STEP = THREAD_PITCH_MM / STEPS_PER_REV;
constexpr unsigned MICROSTEPS = 16;
constexpr double MM_PER_MICRO_STEP = MM_PER_FULL_STEP / MICROSTEPS;
constexpr uint32_t PL_PER_FULL_STEP = static_cast<uint32_t>(UL_PER_MM * MM_PER_FULL_STEP
                                                            * 1'000'000.0);
constexpr uint32_t PL_STEP_VOLUME = PL_PER_FULL_STEP;

/**
 * @brief Descriptive transmission error codes
 * 
 */
enum class Result {
    OK,
    BAD_CRC,
    BUSY,
    PACKET_TOO_SHORT,
    BAD_PAYLOAD_SIZE,
    ERROR_RESPONSE,
    WRITE_ERROR,
    EXTRA_ZEROES,
    INVALID_HEADER,
};

constexpr const char* string_from_result_code(Result result)
{
    switch (result) {
    case Result::BAD_CRC:
        return "BAD_CRC";
    case Result::BAD_PAYLOAD_SIZE:
        return "BAD_PAYLOAD_SIZE";
    case Result::OK:
        return "OK";
    case Result::PACKET_TOO_SHORT:
        return "PACKET_TOO_SHORT";
    case Result::BUSY:
        return "BUSY";
    case Result::ERROR_RESPONSE:
        return "ERROR_RESPONSE";
    case Result::WRITE_ERROR:
        return "WRITE_ERROR";
    case Result::EXTRA_ZEROES:
        return "EXTRA_ZEROES";
    case Result::INVALID_HEADER:
        return "INVALID_HEADER";
    }
    return "???";
}

/**
 * @brief Error code used in protocol responses
 * 
 */
enum class ErrorPayload : u_int8_t {
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

constexpr auto string_from_error_payload(ErrorPayload payload)
{
    switch (payload) {
    case ErrorPayload::OK:
        return "OK";

    case ErrorPayload::INVALID_CRC:
        return "INVALID_CRC";

    case ErrorPayload::INVALID_CMD:
        return "INVALID_CMD";

    case ErrorPayload::WDOG_RESET:
        return "WDOG_RESET";

    case ErrorPayload::MAX_TEMP:
        return "MAX_TEMP";

    case ErrorPayload::MIN_TEMP:
        return "MIN_TEMP";

    case ErrorPayload::ADDRESS_SETUP:
        return "ADDRESS_SETUP";

    case ErrorPayload::CHIP_SELECT:
        return "CHIP_SELECT";

    case ErrorPayload::MICRO_STEP:
        return "MICRO_STEP";

    case ErrorPayload::MOTOR_DIR:
        return "MOTOR_DIR";

    case ErrorPayload::HEATER_TIMEOUT:
        return "HEATER_TIMEOUT";

    case ErrorPayload::HEATER_SENSOR_ERROR:
        return "HEATER_SENSOR_ERROR";

    case ErrorPayload::HEATER_OVERTEMP:
        return "HEATER_OVERTEMP";

    case ErrorPayload::ENDSTOP_TOP_HIT:
        return "ENDSTOP_TOP_HIT";

    case ErrorPayload::ENDSTOP_BOTTOM_HIT:
        return "ENDSTOP_BOTTOM_HIT";

    case ErrorPayload::MOTOR_TIMEOUT:
        return "MOTOR_TIMEOUT";

    case ErrorPayload::HEATER_BROKEN:
        return "HEATER_BROKEN";

    case ErrorPayload::PHYSICS:
        return "PHYSICS";

    case ErrorPayload::HEATER_DISCONNECTED:
        return "HEATER_DISCONNECTED";

    case ErrorPayload::INVALID_PARAM:
        return "INVALID_PARAM";

    case ErrorPayload::VALVE_BROKEN:
        return "VALVE_BROKEN";

    case ErrorPayload::PARAM_STORE:
        return "PARAM_STORE";

    case ErrorPayload::PARAM_LOAD:
        return "PARAM_LOAD";
    }
    return "???";
}

/**
 * @brief Bitflag printhead status wrapper
 * 
 */
struct Status
{
    bool estop_top : 1;
    bool estop_bottom : 1;
    bool is_homing : 1;
    bool dir : 1;
    bool enable : 1;
    bool is_stepping : 1;
    bool heater_active : 1;
    bool fan_active : 1;
    bool is_calibrated : 1;
    bool slider_is_stepping : 1;

    constexpr Status() noexcept
        : Status(0)
    {}
    constexpr Status(uint16_t raw) noexcept
        : estop_top(TEST(raw, ESTOP_TOP))
        , estop_bottom(TEST(raw, ESTOP_BOTTOM))
        , is_homing(TEST(raw, HOMING))
        , dir(TEST(raw, MOTOR_DIR))
        , enable(TEST(raw, MOTOR_ENABLE))
        , is_stepping(TEST(raw, MOTOR_IS_STEPPING))
        , heater_active(TEST(raw, HEATER_ACTIVE))
        , fan_active(TEST(raw, FAN_ACTIVE))
        , is_calibrated(TEST(raw, IS_CALIBRATED))
        , slider_is_stepping(TEST(raw, SLIDER_IS_STEPPING))
    {}

    constexpr auto to_raw() const noexcept -> uint16_t
    {
        uint16_t raw = (estop_top << ESTOP_TOP) | (estop_bottom << ESTOP_BOTTOM)
                       | (is_homing << HOMING) | (dir << MOTOR_DIR) | (enable << MOTOR_ENABLE)
                       | (is_stepping << MOTOR_IS_STEPPING) | (heater_active << HEATER_ACTIVE)
                       | (fan_active << FAN_ACTIVE) | (is_calibrated << IS_CALIBRATED);
        return raw;
    }

private:
    enum Bits {
        ESTOP_TOP = 0,
        ESTOP_BOTTOM = 1,
        HOMING = 2,
        MOTOR_DIR = 3,
        MOTOR_ENABLE = 4,
        MOTOR_IS_STEPPING = 5,
        HEATER_ACTIVE = 6,
        FAN_ACTIVE = 7,
        IS_CALIBRATED = 8,
        SLIDER_IS_STEPPING = 9,
        NUM_STATUS_BITS
    };
};

/**
 * @brief packet parsed from printhead response, and error code if packet could not be fully parsed
 * 
 * @tparam T contained packet payload
 */
template<typename T>
struct Response
{
    Response() = default;
    Packet<T> packet;
    Result result;
};

/**
 * @brief Formatted print of a packet for debugging
 * 
 * @tparam T payload
 * @param packet to print
 */
template<typename T>
void print_packet(const Packet<T>& packet)
{
    SERIAL_ECHO("Packet: { index: ");
    SERIAL_ECHO(static_cast<uint16_t>(packet.ph_index));
    SERIAL_ECHO(", command: ");
    SERIAL_ECHO(static_cast<uint16_t>(packet.command));
    SERIAL_ECHO(", size: ");
    SERIAL_ECHO(packet.payload_size);
    if constexpr (!std::is_void_v<T>) {
        SERIAL_ECHO(", payload: ");
        const auto payload = packet.payload_bytes();
        for (const uint8_t b : payload)
            SERIAL_PRINT(b, PrintBase::Hex);
        SERIAL_CHAR(' ');
    } else {
        SERIAL_ECHO(", (no payload)");
    }
    SERIAL_ECHO(", crc: ");
    SERIAL_PRINT(packet.crc(), PrintBase::Hex);
    SERIAL_ECHOLN(" }");
}

/**
 * @brief Clears any pending incoming bytes in case of bus noise or incomplete transmission
 * 
 * @param serial buffer to flush
 */
void flush_rx(HardwareSerial& serial);

extern millis_t last_serial_activity;
extern size_t printhead_rx_err_counter;

/**
 * @brief validates raw index before conversion
 * 
 * @param index raw from bus
 * @return true 
 * @return false 
 */
constexpr bool valid_index(uint16_t index)
{
    switch (index) {
    case 0:
        [[fallthrough]];
    case 1:
        [[fallthrough]];
    case 2:
        [[fallthrough]];
    case 0xFFFF:
        return true;

    default:
        return false;
    }
}

/**
 * @brief Checks that command is known valid before conversion
 * 
 * @param command raw from bus
 * @return true 
 * @return false 
 */
constexpr bool valid_command(uint16_t command)
{
    if (command >= static_cast<uint16_t>(Command::NOF_CMDS))
        return false;
    switch (command) {
    case 1:
        [[fallthrough]];
    case 2:
        [[fallthrough]];
    case 3:
        [[fallthrough]];
    case 4:
        [[fallthrough]];
    case 5:
        [[fallthrough]];
    case 6:
        [[fallthrough]];
    case 7:
        [[fallthrough]];
    case 8:
        [[fallthrough]];
    case 9:
        [[fallthrough]];
    case 10:
        [[fallthrough]];
    case 11:
        [[fallthrough]];
    case 12:
        [[fallthrough]];
    case 13:
        [[fallthrough]];
    case 14:
        [[fallthrough]];
    case 15:
        [[fallthrough]];
    case 16:
        [[fallthrough]];
    case 17:
        [[fallthrough]];
    case 18:
        [[fallthrough]];
    case 19:
        [[fallthrough]];
    case 20:
        [[fallthrough]];
    case 21:
        [[fallthrough]];
    case 22:
        [[fallthrough]];
    case 23:
        [[fallthrough]];
    case 24:
        [[fallthrough]];
    case 25:
        [[fallthrough]];
    case 26:
        [[fallthrough]];
    case 27:
        [[fallthrough]];
    case 28:
        [[fallthrough]];
    case 29:
        [[fallthrough]];
    case 30:
        [[fallthrough]];
    case 31:
        [[fallthrough]];
    case 32:
        [[fallthrough]];
    case 33:
        [[fallthrough]];
    case 34:
        [[fallthrough]];
    case 35:
        [[fallthrough]];
    case 36:
        [[fallthrough]];
    case 37:
        [[fallthrough]];
    case 38:
        [[fallthrough]];
    case 39:
        [[fallthrough]];
    case 40:
        [[fallthrough]];
    case 41:
        [[fallthrough]];
    case 42:
        [[fallthrough]];
    case 43:
        [[fallthrough]];
    case 44:
        [[fallthrough]];
    case 45:
        [[fallthrough]];
    case 46:
        [[fallthrough]];
    case 47:
        [[fallthrough]];
    case 48:
        [[fallthrough]];
    case 49:
        [[fallthrough]];
    case 50:
        [[fallthrough]];
    case 51:
        [[fallthrough]];
    case 52:
        [[fallthrough]];
    case 53:
        [[fallthrough]];
    case 54:
        [[fallthrough]];
    case 55:
        [[fallthrough]];
    case 56:
        [[fallthrough]];
    case 57:
        [[fallthrough]];
    case 58:
        [[fallthrough]];
    case 59:
        [[fallthrough]];
    case 60:
        [[fallthrough]];
    case 61:
        [[fallthrough]];
    case 62:
        [[fallthrough]];
    case 63:
        [[fallthrough]];
    case 64:
        [[fallthrough]];
    case 65:
        [[fallthrough]];
    case 66:
        [[fallthrough]];
    case 67:
        [[fallthrough]];
    case 68:
        [[fallthrough]];
    case 69:
        [[fallthrough]];
    case 70:
        [[fallthrough]];
    case 71:
        [[fallthrough]];
    case 72:
        [[fallthrough]];
    case 73:
        [[fallthrough]];
    case 74:
        [[fallthrough]];
    case 75:
        [[fallthrough]];
    case 76:
        [[fallthrough]];
    case 77:
        [[fallthrough]];
    case 78:
        [[fallthrough]];
    case 84:
        [[fallthrough]];
    case 99:
        [[fallthrough]];
    case 201:
        [[fallthrough]];
    case 202:
        [[fallthrough]];
    case 203:
        [[fallthrough]];
    case 204:
        [[fallthrough]];
    case 205:
        [[fallthrough]];
    case 206:
        [[fallthrough]];
    case 301:
        [[fallthrough]];
    case 302:
        [[fallthrough]];
    case 401:
        [[fallthrough]];
    case 402:
        [[fallthrough]];
    case 403:
        [[fallthrough]];
    case 404:
        [[fallthrough]];
    case 405:
        [[fallthrough]];
    case 406:
        [[fallthrough]];
    case 407:
        [[fallthrough]];
    case 408:
        [[fallthrough]];
    case 601:
        [[fallthrough]];
    case 602:
        [[fallthrough]];
    case 603:
        [[fallthrough]];
    case 604:
        [[fallthrough]];
    case 605:
        [[fallthrough]];
    case 606:
        [[fallthrough]];
    case 607:
        [[fallthrough]];
    case 608:
        [[fallthrough]];
    case 609:
        [[fallthrough]];
    case 610:
        [[fallthrough]];
    case 611:
        [[fallthrough]];
    case 612:
        [[fallthrough]];
    case 613:
        [[fallthrough]];
    case 801:
        [[fallthrough]];
    case 802:
        [[fallthrough]];
    case 803:
        [[fallthrough]];
    case 804:
        [[fallthrough]];
    case 805:
        [[fallthrough]];
    case 806:
        [[fallthrough]];
    case 807:
        [[fallthrough]];
    case 808:
        [[fallthrough]];
    case 809:
        [[fallthrough]];
    case 1001:
        [[fallthrough]];
    case 1006:
        [[fallthrough]];
    case 1007:
        [[fallthrough]];
    case 1008:
        [[fallthrough]];
    case 1009:
        [[fallthrough]];
    case 1010:
        [[fallthrough]];
    case 1011:
        [[fallthrough]];
    case 1016:
        return true;
    default:
        return false;
    }
}

/**
 * @brief validate packet size
 * 
 * @param size raw from bus
 * @return true 
 * @return false 
 */
constexpr bool valid_size(uint16_t size)
{
    return (size <= 0x00ff);
}

/**
 * @brief High level checked method to read a response packet from bus
 * 
 * @tparam T packet payload
 * @param serial 
 * @param enable_debug 
 * @return Response<T> 
 */
template<typename T>
Response<T> receive(HardwareSerial& serial, bool enable_debug = true)
{
    // header + crc
    static constexpr size_t EMPTY_PACKET_SIZE = sizeof(EmptyPacket) + sizeof(uint16_t);

    static constexpr size_t EXPECTED_PACKET_SIZE = []() {
        if constexpr (std::is_same_v<T, void>) {
            return EMPTY_PACKET_SIZE;
        } else {
            // using sizeof(Packet<T>) can cause issues due to alignment
            static_assert(sizeof(T) < 128, "Packet payload type too large for buffer!");
            return EMPTY_PACKET_SIZE + sizeof(T);
        }
    }();
    uint8_t packet_buffer[EXPECTED_PACKET_SIZE + 2]{}; // larger than needed in case of transceiver defects

    Packet<T> incoming{};

    static auto err = [&incoming, enable_debug](Result code) -> Response<T> {
        ++printhead_rx_err_counter;
        if (DEBUGGING(ERRORS) && enable_debug)
            SERIAL_ECHOLNPGM("CHANT_RX_ERR:", string_from_result_code(code));
        return Response<T>{incoming, code};
    };
    serial.setTimeout(25);
    size_t bytes_received = serial.readBytes(packet_buffer, EXPECTED_PACKET_SIZE);

    // validation
    uint16_t test_index, test_command, test_size;
    memcpy(&test_index, &packet_buffer[0], 2);
    memcpy(&test_command, &packet_buffer[2], 2);
    memcpy(&test_size, &packet_buffer[4], 2);
    bool valid = valid_index(test_index) && valid_command(test_command) && valid_size(test_size);

    bool got_extra_zeroes = false;
    if (!valid || test_command == static_cast<uint16_t>(Command::ACK)) {
        memcpy(&test_index, &packet_buffer[1], 2);
        memcpy(&test_command, &packet_buffer[3], 2);
        memcpy(&test_size, &packet_buffer[5], 2);
        bool valid_with_zeroes = valid_index(test_index) && valid_command(test_command)
                                 && valid_size(test_size);
        if (valid_with_zeroes) {
            millis_t timeout = millis() + 5;
            int leftover = -1;
            while (leftover < 0 && millis() < timeout)
                leftover = serial.read();
            if (leftover >= 0) {
                packet_buffer[bytes_received++] = static_cast<uint8_t>(leftover);
                flush_rx(serial);
                got_extra_zeroes = true;
                valid = true;
                err(Result::EXTRA_ZEROES);
            }
        }
    }

    last_serial_activity = millis();

    if (DEBUGGING(INFO) && enable_debug) {
        SERIAL_ECHO("Bytes received: [ ");
        for (size_t i = 0; i < bytes_received; ++i) {
            if (i == 0 && got_extra_zeroes)
                continue;
            SERIAL_PRINT(packet_buffer[i], PrintBase::Hex);
            SERIAL_CHAR(' ');
        }
        SERIAL_ECHOLN("]");
    }
    if (bytes_received < EXPECTED_PACKET_SIZE)
        return err(Result::PACKET_TOO_SHORT);

    if (!valid)
        return err(Result::INVALID_HEADER);

    size_t packet_index = got_extra_zeroes ? 1 : 0; // usually has a leading 0 byte

    // indexes starting at 1 because there is always a leading zero
    memcpy(&incoming.ph_index, &packet_buffer[packet_index], 2);
    packet_index += 2;
    memcpy(&incoming.command, &packet_buffer[packet_index], 2);
    packet_index += 2;
    memcpy(&incoming.payload_size, &packet_buffer[packet_index], 2);
    packet_index += 2;

    if (incoming.command == Command::ERROR) {
        //const uint8_t e = static_cast<uint8_t>(serial.read());
        if (DEBUGGING(ERRORS)) {
            SERIAL_ECHOLNPGM("RECD_CHANT_ERROR: ",
                             string_from_error_payload(
                                 static_cast<ErrorPayload>(packet_buffer[packet_index])));
        }
        flush_rx(serial);
        ++printhead_rx_err_counter;
        return err(Result::ERROR_RESPONSE);
    }

    if (static_cast<size_t>(incoming.payload_size + EMPTY_PACKET_SIZE) > (EXPECTED_PACKET_SIZE + 2)
        || incoming.payload_size > bytes_received) {
        return err(Result::BAD_PAYLOAD_SIZE);
    }

    uint16_t crc;
    if constexpr (!std::is_void_v<T>) {
        memcpy(&incoming.payload, &packet_buffer[packet_index], sizeof(incoming.payload));
        packet_index += sizeof(incoming.payload);
    }
    memcpy(&crc, &packet_buffer[packet_index], 2);

    constexpr static bool allow_bad_crc = false;
    if (!allow_bad_crc && crc != incoming.crc())
        return err(Result::BAD_CRC);

    if (DEBUGGING(INFO) && enable_debug) {
        SERIAL_ECHO("Parsed ");
        print_packet(incoming);
    }

    return Response<T>{incoming, Result::OK};

    // ACK would go here
}

extern size_t printhead_tx_err_counter;

template<typename T>
Result send(const Packet<T>& request,
            HardwareSerial& serial,
            bool expect_ack = true,
            bool enable_debug = true)
{
    // make sure at least a millisecond has passed between sends
    constexpr static millis_t MIN_CHANT_SEND_DELAY = 10;
    if (millis() <= last_serial_activity + MIN_CHANT_SEND_DELAY)
        delay(MIN_CHANT_SEND_DELAY);

    static auto err = [](Result code) {
        ++printhead_tx_err_counter;
        last_serial_activity = millis();
        return code;
    };

    if (DEBUGGING(INFO) && enable_debug) {
        SERIAL_ECHO("Sending ");
        print_packet(request);
    }
    const auto packet_bytes = request.bytes();
    flush_rx(serial);
    if (DEBUGGING(INFO) && enable_debug)
        SERIAL_ECHO("bytes sent: [ ");
    OUT_WRITE(CHANT_RTS_PIN, HIGH);
    for (const auto byte : packet_bytes) {
        if (DEBUGGING(INFO) && enable_debug) {
            SERIAL_PRINT(byte, PrintBase::Hex);
            SERIAL_CHAR(' ');
        }
        serial.write(byte);
    }
    serial.flush();
    WRITE(CHANT_RTS_PIN, LOW);
    last_serial_activity = millis();
    if (DEBUGGING(INFO) && enable_debug)
        SERIAL_ECHOLN("]");
    if (serial.getWriteError())
        return err(Result::WRITE_ERROR);
    // should read an ACK after this write to assure complete transaction
    if (expect_ack)
        return receive<void>(serial).result;

    return Result::OK;
}

/**
 * @brief send without type safety or bounds checks
 * 
 * @param data type erased pointer to buffer
 * @param size bytes to read from buffer onto the bus
 * @param serial 
 * @return Result 
 */
Result unsafe_send(const void* data, const size_t size, HardwareSerial& serial);

/**
 * @brief high level transaction command to send and recieve type checked packets with error reporting
 * 
 * @tparam OUT expected response payload
 * @tparam IN provided request payload
 * @param packet to send
 * @param serial 
 * @param enable_debug 
 * @return Response<OUT> 
 */
template<typename OUT, typename IN = void>
Response<OUT> send_and_receive(const Packet<IN>& packet,
                               HardwareSerial& serial,
                               bool enable_debug = true)
{
    Response<OUT> response;
    response.result = send<IN>(packet, serial, false, enable_debug);
    if (response.result != Result::OK)
        return response;
    return receive<OUT>(serial, enable_debug);
}

enum class ExtruderDirection : uint8_t {
    Extrude,
    Retract
};

enum class SliderDirection : uint8_t {
    Pull,
    Push
};

namespace constants {
static constexpr size_t CS_FANS = 2;
static constexpr size_t CS_TEMS = 2;
static constexpr size_t CS_ENCODERS = 6;
static constexpr size_t FW_VERSION_LEN = 12;
static constexpr size_t PID_PARAMS = 3;
} // namespace constants

namespace {
using FanSpeeds = std::array<uint16_t, constants::CS_FANS>;
using TemTemps = std::array<int16_t, constants::CS_TEMS>;
using EncoderStates = std::array<int32_t, constants::CS_ENCODERS>;
using FirmwareVersion = std::array<char, constants::FW_VERSION_LEN>;
using PIDParams = std::array<uint16_t, constants::PID_PARAMS>;

} // namespace

enum class EncoderIndex {
    SliderOne,
    ExtruderOne,
    SliderTwo,
    ExtruderTwo,
    SliderThree,
    ExtruderThree
};

/**
 * @brief Get the encoder state object
 * 
 * @param states 
 * @param index 
 * @return constexpr int32_t 
 */
constexpr int32_t get_encoder_state(const EncoderStates& states, EncoderIndex index)
{
    return states[static_cast<size_t>(index)];
}

/**
 * @brief local bundle of synchronized printhead state
 * 
 */
struct PrintheadState
{
    int32_t extruder_encoder;
    int32_t slider_pos;
    int32_t slider_encoder;
    FanSpeeds fan_set_speeds;
    TemTemps tem_set_temps;
    uint16_t raw_temperature;
    Status status;
    bool extruder_is_homed;
    bool slider_is_homed;
};

#if ENABLED(AUTO_REPORT_CHANTARELLE)
namespace reporters {
void tick_all();
struct State : AutoReporter<State>
{
    static void report();
};
} // namespace reporters
#endif

/**
 * @brief High level controller to handle protocol and state management
 * 
 */
class Controller
{
    HardwareSerial& bus;
    std::array<PrintheadState, EXTRUDERS> ph_states{};

public:
    bool disable_background_updates;

    // initialization
    constexpr explicit Controller(HardwareSerial& ph_bus)
        : bus(ph_bus)
    {}

    /**
     * @brief setup required pins and bus configuration
     * 
     */
    void init();

    /**
     * @brief method to call on tool change to ensure printhead synchronization
     * 
     * @param tool_index 
     */
    void tool_change(uint8_t tool_index);

    /**
     * @brief call in idle to handle background requests
     * 
     */
    void update(millis_t now = millis());

    /**
     * @brief pretty print current states
     * 
     */
    void report_states();

    /**
     * @brief Get the latest extruder temp object
     * 
     * @param index 
     * @return celsius_float_t 
     */
    celsius_float_t get_latest_extruder_temp(Index index);

    /**
     * @brief poll internal state for ALL extruder moving status
     * 
     * @return true 
     * @return false 
     */
    bool extruder_busy();

    /**
     * @brief poll internal state for selected moving status
     * 
     * @param index 
     * @return true 
     * @return false 
     */
    bool extruder_busy(Index index);

    /**
     * @brief poll internal state for ALL moving status
     * 
     * @return true 
     * @return false 
     */
    bool slider_busy();

    /**
     * @brief poll internal state for selected moving status
     * 
     * @param index 
     * @return true 
     * @return false 
     */
    bool slider_busy(Index index);

    // Metadata methods
    Response<void> get_info(Index index);
    Response<FirmwareVersion> get_fw_version(Index index);
    Response<void> get_all(Index index);
    Response<std::array<uint8_t, 12>> get_uuid(Index index);
    Response<Status> get_status(Index index, bool debug = true);

    // Temperature methods
    Response<uint16_t> set_temperature(Index index, celsius_float_t temperature);
    Response<uint16_t> get_temperature(Index index, bool debug = true);
    Response<PIDParams> set_pid(Index index, float p, float i, float d);
    Response<std::array<uint16_t, 3>> get_pid(Index index);
    Result set_fan_speed(Index index, FanSpeeds fan_speeds);
    Response<FanSpeeds> get_fan_speed(Index index);
    auto set_tem_debug(Index index, TemTemps tem_pwms) -> Result;
    auto get_tem_debug(Index index) -> Response<TemTemps>;

    // Extruder Stepper driver methods
    Result set_extrusion_speed(Index index, uint32_t feedrate);
    Response<uint32_t> get_extrusion_speed(Index index);
    Result set_extruder_stallguard_threshold(Index index, uint8_t threshold);
    Response<uint8_t> get_extruder_stallguard_threshold(Index index);
    Result set_extruder_microsteps(Index index, uint8_t microsteps);
    Response<uint8_t> get_extruder_microsteps(Index index);
    Result set_extruder_rms_current(Index index, uint16_t mA);
    Response<uint16_t> get_extruder_rms_current(Index index);
    Result set_extruder_hold_current(Index index, uint16_t mA);
    Response<uint16_t> get_extruder_hold_current(Index index);
    Result home_extruder(Index index, ExtruderDirection direction);
    Result start_extruding(Index index);
    Result stop_extruding(Index index);
    Result add_raw_extruder_steps(Index index, int32_t steps);
    Result extruder_move(Index index, float uL);
    Result set_extruder_direction(Index index, bool direction);

    // Slider Valve driver methods
    Result set_valve_speed(Index index, feedRate_t feedrate);
    Response<uint32_t> get_valve_speed(Index index);
    Result set_valve_stallguard_threshold(Index index, uint8_t threshold);
    Response<uint8_t> get_valve_stallguard_threshold(Index index);
    Result set_valve_microsteps(Index index, uint8_t microsteps);
    Response<uint8_t> get_valve_microsteps(Index index);
    Result set_valve_rms_current(Index index, uint16_t mA);
    Response<uint16_t> get_valve_rms_current(Index index);
    Result set_valve_hold_current(Index index, uint16_t mA);
    Result home_slider_valve(Index index, SliderDirection dir);
    Result move_slider_valve(Index index, int32_t steps);
    void stop_active_extrudes();
    Result set_volume_per_fullstep(Index index, uint32_t picoliters);
    Result set_step_volume(Index index, uint32_t picoliters);
    Response<uint32_t> get_step_volume(Index index);
    Response<EncoderStates> debug_get_encoders(bool debug = true);
    Result disable_heating(Index index);
    Response<TemTemps> debug_get_temperature(Index index, bool debug = true);
};

} // namespace printhead