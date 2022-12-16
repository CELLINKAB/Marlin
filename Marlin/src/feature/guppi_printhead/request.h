// Copyright Cellink 2022 GPLv3

#pragma once

#include "../../inc/MarlinConfig.h"

#include <array>

#include "packet.h"

namespace printhead {

enum class Result {
    OK,
    BAD_CRC,
    BUSY,
    PACKET_TOO_SHORT,
    BAD_PAYLOAD_SIZE,
    UNIMPLEMENTED
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
    case Result::UNIMPLEMENTED:
        return "UNIMPLEMENTED";
    }
    __unreachable();
}

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

template<typename T>
struct Response
{
    Response() = default;
    Packet<T> packet;
    Result result;
};

template<typename T>
void print_packet(const Packet<T>& packet)
{
    SERIAL_PRINT(static_cast<uint16_t>(packet.ph_index), PrintBase::Hex);
    SERIAL_CHAR(' ');

    SERIAL_PRINT(static_cast<uint16_t>(packet.command), PrintBase::Hex);
    SERIAL_CHAR(' ');

    SERIAL_PRINT(packet.payload_size, PrintBase::Hex);
    SERIAL_CHAR(' ');

    if constexpr (!std::is_void_v<T>) {
        const auto payload = packet.payload_bytes();
        for (const uint8_t b : payload)
            SERIAL_PRINT(b, PrintBase::Hex);
        SERIAL_CHAR(' ');
    }

    SERIAL_PRINTLN(packet.crc, PrintBase::Hex);
}

template<typename T>
void print_response(Response<T> response)
{
    if (response.result != Result::OK) {
        SERIAL_ECHOLNPGM("ERR:", string_from_result_code(response.result));
    }

    print_packet(response.packet);
}

void flush_rx(HardwareSerial& serial);

template<typename T>
Response<T> receive(HardwareSerial& serial)
{
    // this seems bug-prone...
    static constexpr size_t MAX_PACKET = 128;
    static uint8_t packet_buffer[MAX_PACKET]{};

    Packet<T> incoming;

    auto bytes_received = serial.readBytes(packet_buffer, 6);

    if (bytes_received < 6)
        return Response<T>{incoming, Result::PACKET_TOO_SHORT};

    memcpy(&incoming.ph_index, &packet_buffer[0], 2);
    memcpy(&incoming.command, &packet_buffer[2], 2);
    memcpy(&incoming.payload_size, &packet_buffer[4], 2);

    if (incoming.command == Command::ERROR) {
        const uint8_t e = static_cast<uint8_t>(serial.read());
        if (DEBUGGING(ERRORS)) {
            SERIAL_ECHO("RECD_CHANT_ERROR: ");
            SERIAL_PRINTLN(e, PrintBase::Dec);
        }
        flush_rx(serial);
        return Response<T>{incoming, Result::UNIMPLEMENTED};
    }

    if ((incoming.payload_size + 2) > MAX_PACKET)
        return Response<T>{incoming, Result::BAD_PAYLOAD_SIZE};

    bytes_received = serial.readBytes(packet_buffer, incoming.payload_size + 2);
    if (incoming.payload_size != bytes_received - 2)
        return Response<T>{incoming, Result::BAD_PAYLOAD_SIZE};

    if constexpr (!std::is_void_v<T>) {
        memcpy(&incoming.payload, packet_buffer, sizeof(incoming.payload));
        memcpy(&incoming.crc, &packet_buffer[incoming.payload_size], 2);
    }
    uint16_t crc = calculate_crc16(incoming.bytes());

    if (crc != incoming.crc)
        return Response<T>{incoming, Result::BAD_CRC};

    return Response<T>{incoming, Result::OK};

    // ACK would go here
}

template<typename T>
Result send(const Packet<T>& request, HardwareSerial& serial, bool expect_ack = true)
{
    static auto init [[maybe_unused]] = [] {
        OUT_WRITE(CHANT_RTS_PIN, LOW);
        return 0;
    }();
    const auto packet_bytes = request.bytes();
    flush_rx(serial);
    WRITE(CHANT_RTS_PIN, HIGH);
    for (const auto byte : packet_bytes)
        serial.write(byte);
    serial.flush();
    WRITE(CHANT_RTS_PIN, LOW);

    // should read an ACK after this write to assure complete transaction
    if (expect_ack)
        auto _ = receive<void>(serial);
    return Result::OK;
}

Result unsafe_send(const void* data, const size_t size, HardwareSerial& serial);

template<typename OUT, typename IN = void>
Response<OUT> send_and_receive(const Packet<IN>& packet, HardwareSerial& serial)
{
    Response<OUT> response;
    response.result = send<IN>(packet, serial, false);
    if (response.result != Result::OK)
        return response;
    return receive<OUT>(serial);
}

enum class ExtruderDirection : uint8_t {
    Extrude,
    Retract
};

enum class SliderDirection : uint8_t {
    Push,
    Pull
};

namespace constants {
static constexpr size_t CS_FANS = 2;
static constexpr size_t CS_TEMS = 2;
} // namespace constants

namespace {
using FanSpeeds = std::array<uint16_t, constants::CS_FANS>;
using TemTemps = std::array<uint16_t, constants::CS_TEMS>;
} // namespace

struct PrintheadState
{
    int32_t extruder_pos;
    int32_t slider_pos;
    FanSpeeds fan_set_speeds;
    TemTemps tem_set_temps;
    bool extruder_is_homed;
    bool slider_is_homed;
    bool is_currently_extruding;
};

class Controller
{
    HardwareSerial& bus;
    std::array<PrintheadState, EXTRUDERS> ph_states;

    void set_extruder_state(Index index, bool state);

public:
    // initialization
    Controller(HardwareSerial& ph_bus)
        : bus(ph_bus)
    {}
    void init();

    // Metadata methods
    Response<void> get_info(Index index);
    Response<void> get_fw_version(Index index);
    Response<void> get_all(Index index);
    Response<std::array<uint8_t, 12>> get_uuid(Index index);
    Response<void> get_status(Index index);
    // Temperature methods
    Result set_temperature(Index index, celsius_t temperature);
    celsius_float_t get_temperature(Index index);
    Result set_pid(Index index, float p, float i, float d);
    Response<std::array<uint16_t, 3>> get_pid(Index index);
    Result set_fan_speed(Index index, FanSpeeds fan_speeds);
    Response<FanSpeeds> get_fan_speed(Index index);
    auto set_tem_debug(Index index, TemTemps tem_pwms) -> Result;
    auto get_tem_debug(Index index) -> Response<TemTemps>;
    // Extruder Stepper driver methods
    Result set_extrusion_speed(Index index, feedRate_t feedrate);
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
};

} // namespace printhead