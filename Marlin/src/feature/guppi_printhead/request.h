// Copyright Cellink 2022 GPLv3

#pragma once

#include "../../inc/MarlinConfig.h"

#include <array>

#include "packet.h"

#define AUTO_REPORT_CHANTARELLE
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

enum class Result {
    OK,
    BAD_CRC,
    BUSY,
    PACKET_TOO_SHORT,
    BAD_PAYLOAD_SIZE,
    UNIMPLEMENTED,
    WRITE_ERROR
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
    case Result::WRITE_ERROR:
        return "WRITE_ERROR";
    }
    __unreachable();
}

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

template<typename T>
void print_response(Response<T> response)
{
    if (response.result != Result::OK) {
        SERIAL_ECHOLNPGM("ERR:", string_from_result_code(response.result));
    }

    print_packet(response.packet);
}

void flush_rx(HardwareSerial& serial);

extern millis_t last_send;

template<typename T>
Response<T> receive(HardwareSerial& serial, bool enable_debug = true)
{
    // header + crc + 1 byte padding on either side
    static constexpr size_t EMPTY_PACKET_SIZE = sizeof(EmptyPacket) + sizeof(uint16_t);

    static constexpr size_t MAX_PACKET = []() {
        if constexpr (std::is_same_v<T, void>) {
            return EMPTY_PACKET_SIZE;
        } else {
            // using sizeof(Packet<T>) can cause issues due to alignment
            static_assert(sizeof(T) < 128, "Packet payload type too large for buffer!");
            return EMPTY_PACKET_SIZE + sizeof(T);
        }
    }();
    uint8_t packet_buffer[MAX_PACKET + 2]{}; // larger than needed in case of transceiver defects

    Packet<T> incoming{};

    serial.setTimeout(10);
    auto bytes_received = serial.readBytes(packet_buffer, MAX_PACKET);
    bool got_extra_zeroes = false;
    if (serial.available() > 0) {
        packet_buffer[MAX_PACKET] = static_cast<uint8_t>(serial.read());
        ++bytes_received;
        flush_rx(serial);
        got_extra_zeroes = true;
    }
    if (DEBUGGING(INFO) && enable_debug) {
        SERIAL_ECHO("Bytes received: [ ");
        for (size_t i = 0; i < bytes_received; ++i) {
            if (i == 0 && got_extra_zeroes) continue;
            SERIAL_PRINT(packet_buffer[i], PrintBase::Hex);
            SERIAL_CHAR(' ');
        }
        SERIAL_ECHOLN("]");
    }
    if (bytes_received < 8)
        return Response<T>{incoming, Result::PACKET_TOO_SHORT};

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
            SERIAL_ECHO("RECD_CHANT_ERROR: ");
            SERIAL_PRINTLN(packet_buffer[packet_index], PrintBase::Dec);
        }
        flush_rx(serial);
        return Response<T>{incoming, Result::UNIMPLEMENTED};
    }

    if (static_cast<size_t>(incoming.payload_size + 8) > MAX_PACKET)
        return Response<T>{incoming, Result::BAD_PAYLOAD_SIZE};

    if (incoming.payload_size > bytes_received)
        return Response<T>{incoming, Result::BAD_PAYLOAD_SIZE};

    uint16_t crc;
    if constexpr (!std::is_void_v<T>) {
        memcpy(&incoming.payload, &packet_buffer[packet_index], sizeof(incoming.payload));
        packet_index += sizeof(incoming.payload);
    }
    memcpy(&crc, &packet_buffer[packet_index], 2);

    constexpr static bool allow_bad_crc = false;
    if (!allow_bad_crc && crc != incoming.crc())
        return Response<T>{incoming, Result::BAD_CRC};

    if (DEBUGGING(INFO) && enable_debug) {
        SERIAL_ECHOLN("Parsed:");
        print_packet(incoming);
    }

    last_send = millis();
    return Response<T>{incoming, Result::OK};

    // ACK would go here
}

template<typename T>
Result send(const Packet<T>& request,
            HardwareSerial& serial,
            bool expect_ack = true,
            bool enable_debug = true)
{
    // make sure at least a millisecond has passed between sends
    constexpr static millis_t MIN_CHANT_SEND_DELAY = 2;
    while (millis() <= last_send + MIN_CHANT_SEND_DELAY)
        delay(1);

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
    last_send = millis();
    if (DEBUGGING(INFO) && enable_debug)
        SERIAL_ECHOLN("]");
    if (serial.getWriteError())
        return Result::WRITE_ERROR;
    // should read an ACK after this write to assure complete transaction
    if (expect_ack)
        return receive<void>(serial).result;

    return Result::OK;
}

Result unsafe_send(const void* data, const size_t size, HardwareSerial& serial);

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
} // namespace constants

namespace {
using FanSpeeds = std::array<uint16_t, constants::CS_FANS>;
using TemTemps = std::array<uint16_t, constants::CS_TEMS>;
using EncoderStates = std::array<int32_t, constants::CS_ENCODERS>;
using FirmwareVersion = std::array<char, constants::FW_VERSION_LEN>;
} // namespace

enum class EncoderIndex {
    SliderOne,
    ExtruderOne,
    SliderTwo,
    ExtruderTwo,
    SliderThree,
    ExtruderThree
};

constexpr int32_t get_encoder_state(const EncoderStates& states, EncoderIndex index)
{
    return states[static_cast<size_t>(index)];
}

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

class Controller
{
    HardwareSerial& bus;
    std::array<PrintheadState, EXTRUDERS> ph_states{};

public:
    // initialization
    Controller(HardwareSerial& ph_bus)
        : bus(ph_bus)
    {}

    void init();

    void tool_change(uint8_t tool_index);

    void update();

    void report_states();

    celsius_float_t get_latest_extruder_temp(Index index);

    bool extruder_busy();
    bool extruder_busy(Index index);
    bool slider_busy();
    bool slider_busy(Index index);

    // Metadata methods
    Response<void> get_info(Index index);
    Response<FirmwareVersion> get_fw_version(Index index);
    Response<void> get_all(Index index);
    Response<std::array<uint8_t, 12>> get_uuid(Index index);
    Response<Status> get_status(Index index, bool debug = true);
    // Temperature methods
    Result set_temperature(Index index, celsius_t temperature);
    Response<uint16_t> get_temperature(Index index, bool debug = true);
    Result set_pid(Index index, float p, float i, float d);
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
};

} // namespace printhead