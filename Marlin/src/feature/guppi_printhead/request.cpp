

#include "request.h"

#include "../../MarlinCore.h"

using namespace printhead;

void printhead::print_response(Response response)
{
    if (response.result != Result::OK) {
        SERIAL_ECHOLNPGM("ERR:", string_from_result_code(response.result));
    }

    response.packet.print();
}

Result printhead::send(const Packet& request, HardwareSerial& serial, bool expect_ack)
{
    static auto init [[maybe_unused]] = [] {
        OUT_WRITE(CHANT_RTS_PIN, LOW);
        return 0;
    }();
    const auto header = request.header_bytes();
    const auto crc = request.crc_bytes();
    WRITE(CHANT_RTS_PIN, HIGH);
    serial.write(header.data(), header.size());
    serial.write(request.payload, request.payload_size);
    serial.write(crc.data(), crc.size());
    serial.flush();
    WRITE(CHANT_RTS_PIN, LOW);

    // should read an ACK after this write to assure complete transaction
    if (expect_ack) auto _ = receive(serial);
    return Result::OK;
}

Response printhead::receive(HardwareSerial& serial)
{
    // this seems bug-prone...
    static constexpr size_t MAX_PACKET = 128;
    static uint8_t packet_buffer[MAX_PACKET]{};

    Packet incoming;

    auto bytes_received = serial.readBytes(packet_buffer, 6);

    if (bytes_received < 6)
        return Response{incoming, Result::PACKET_TOO_SHORT};

    memcpy(&incoming.ph_index, &packet_buffer[0], 2);
    memcpy(&incoming.command, &packet_buffer[2], 2);
    memcpy(&incoming.payload_size, &packet_buffer[4], 2);

    if ((incoming.payload_size + 2) > MAX_PACKET)
        return Response{incoming, Result::BAD_PAYLOAD_SIZE};

    bytes_received = serial.readBytes(packet_buffer, incoming.payload_size + 2);
    if (incoming.payload_size != bytes_received - 2)
        return Response{incoming, Result::BAD_PAYLOAD_SIZE};

    incoming.payload = packet_buffer;
    memcpy(&incoming.crc, &packet_buffer[incoming.payload_size], 2);
    uint16_t crc = crc16_from_bytes(incoming.payload, incoming.payload_size);

    if (crc != incoming.crc)
        return Response{incoming, Result::BAD_CRC};

    return Response{incoming, Result::OK};

    // ACK would go here
}

Response printhead::send_and_receive(Packet packet, HardwareSerial& serial)
{
    Response response;
    response.result = send(packet, serial, false);
    if (response.result != Result::OK)
        return response;
    return receive(serial);
}

Packet::HeaderBytes Packet::header_bytes() const
{
    static_assert(sizeof(ph_index) + sizeof(command) + sizeof(payload_size) == sizeof(HeaderBytes),
                  "Packet metadata sizing incorrect");
    HeaderBytes bytes;
    auto* ptr = bytes.data();

    memcpy(ptr, &ph_index, 2);
    memcpy(ptr + 2, &command, 2);
    memcpy(ptr + 4, &payload_size, 2);
    return bytes;
}

Packet::CrcBytes Packet::crc_bytes() const
{
    static_assert(sizeof(crc) == sizeof(CrcBytes), "Packet crc sizing incorrect");
    CrcBytes bytes;
    memcpy(bytes.data(), &crc, 2);
    return bytes;
}

void Packet::print() const
{
    SERIAL_PRINT(uint16_t(ph_index), PrintBase::Hex);
    SERIAL_CHAR(' ');

    SERIAL_PRINT(uint16_t(command), PrintBase::Hex);
    SERIAL_CHAR(' ');

    SERIAL_PRINT(uint16_t(payload_size), PrintBase::Hex);
    SERIAL_CHAR(' ');

    if (payload) {
        for (size_t i = 0; i < payload_size; ++i)
            SERIAL_PRINT(uint8_t(payload[i]), PrintBase::Hex);
        SERIAL_CHAR(' ');
    }

    SERIAL_PRINTLN(crc, PrintBase::Hex);
}

void Controller::set_extruder_state(printhead::Index index, bool state)
{
    switch (index) {
    case printhead::Index::One:
        [[fallthrough]];
    case printhead::Index::Two:
        [[fallthrough]];
    case printhead::Index::Three:
        ph_states[static_cast<size_t>(index)].is_currently_extruding = state;
        break;
    case printhead::Index::All:
        for (auto& ph_state : ph_states)
            ph_state.is_currently_extruding = state;
        break;
    default:
        return;
    }
}

void Controller::init()
{
    constexpr static unsigned CHANT_BAUDRATE = 115200;
    bus.begin(CHANT_BAUDRATE);
}

Result Controller::set_temperature(Index index, celsius_t temperature)
{
    uint16_t chant_temp = temperature + 30'000;
    Packet request(index, Command::SET_TEMP, &temperature, sizeof(temperature));
    return send(request, bus);
}

celsius_float_t Controller::get_temperature(Index index) {
    Packet request(index, Command::GET_MEASURED_TEMP);
    auto res = send_and_receive(request, bus);
    if (res.result != Result::OK || res.packet.payload_size < 2) return 0.0f;
    uint16_t chant_temp;
    memcpy(&chant_temp, res.packet.payload, sizeof(chant_temp));
    return (chant_temp - 30'000) / 100.0f;
}

Response Controller::get_info(Index index)
{
    Packet packet(index, Command::GET_DEVICE_INFO);
    return send_and_receive(packet, bus); // TODO: parse response into type
}

Response Controller::get_fw_version(Index index)
{
    Packet packet(index, Command::GET_SW_VERSION);
    return send_and_receive(packet, bus); //TODO: parse result into relevant type
}

Result Controller::set_pid(Index index, float p, float i, float d)
{
    static constexpr uint8_t PAYLOAD_SIZE = 6;
    uint16_t p_ = static_cast<uint16_t>(p * 100);
    uint16_t i_ = static_cast<uint16_t>(i * 100);
    uint16_t d_ = static_cast<uint16_t>(d * 100);

    uint8_t payload[PAYLOAD_SIZE];
    memcpy(payload, &p_, 2);
    memcpy(payload + 2, &i_, 2);
    memcpy(payload + 4, &d_, 2);

    Packet packet(index, Command::SET_PID, payload, PAYLOAD_SIZE);
    return send(packet, bus);
}

Response Controller::get_pid(Index index)
{
    Packet packet(index, Command::GET_PID);
    return send_and_receive(packet, bus); // TODO: parse incoming response and return payload values
}

auto Controller::set_fan_speed(Index index, FanSpeeds fan_speeds) -> Result
{
    Packet packet(index, Command::DEBUG_SET_FAN_PWM, fan_speeds.data(), sizeof(fan_speeds));
    auto res = send(packet, bus);
    // if (res == Result::OK) // TODO: handle printhead state
    return res;
}

auto Controller::get_fan_speed(Index index) -> Response
{
    Packet packet(index, Command::DEBUG_GET_FAN_PWM);
    return send_and_receive(packet, bus); // TODO: handle printhead state, get TACH from chant
}

auto Controller::set_tem_debug(Index index, TemTemps tem_pwms) -> Result {
    Packet packet(index, Command::DEBUG_SET_TEM_PWM, tem_pwms.data(), sizeof(tem_pwms));
    return send(packet, bus);
}
auto Controller::get_tem_debug(Index index) -> Response {
    Packet packet(index, Command::DEBUG_GET_TEM_PWM);
    return send_and_receive(packet, bus);
}

Result Controller::set_extrusion_speed(Index index, feedRate_t feedrate)
{
    constexpr double filament_radius = DEFAULT_NOMINAL_FILAMENT_DIA / 2.0;
    constexpr double mm_to_pl_factor = (filament_radius * filament_radius * PI) * 1000.0;
    const uint32_t feedrate_pl_s = static_cast<uint32_t>(feedrate * mm_to_pl_factor);
    Packet packet(index, Command::SET_EXTRUSION_SPEED, &feedrate_pl_s, sizeof(feedrate_pl_s));
    return send(packet, bus);
}
Response Controller::get_extrusion_speed(Index index)
{
    Packet packet(index, Command::GET_EXTRUSION_SPEED);
    return send_and_receive(packet, bus);
}
Result Controller::set_extruder_stallguard_threshold(Index index, uint8_t threshold)
{
    Packet packet(index, Command::SYRINGEPUMP_SET_ESTOP_THRESH, &threshold, 1);
    return send(packet, bus);
}
Response Controller::get_extruder_stallguard_threshold(Index index)
{
    Packet packet(index, Command::SYRINGEPUMP_GET_ESTOP_THRESH);
    return send_and_receive(packet, bus);
}
Result Controller::set_extruder_microsteps(Index index, uint8_t microsteps)
{
    Packet packet(index, Command::SET_MICROSTEP, &microsteps, 1);
    return send(packet, bus);
}
Response Controller::get_extruder_microsteps(Index index)
{
    Packet packet(index, Command::GET_MICROSTEP);
    return send_and_receive(packet, bus);
}
Result Controller::set_extruder_rms_current(Index index, uint16_t mA) {}
Response Controller::get_extruder_rms_current(Index index) {}
Result Controller::set_extruder_hold_current(Index index, uint16_t mA) {}
Response Controller::get_extruder_hold_current(Index index) {}
Result Controller::home_extruder(Index index, ExtruderDirection direction)
{
    Packet packet(index, Command::MOVE_TO_HOME_POSITION, &direction, sizeof(direction));
    auto res = send(packet, bus);

    if (res == Result::OK)
        ph_states[static_cast<uint8_t>(index)].extruder_is_homed = true;
    return res;
}
Result Controller::set_extruder_direction(Index index, bool direction)
{
    return send(Packet(index, Command::SET_EXTRUSION_DIRECTION), bus);
}
Result Controller::extruder_move(Index index, float uL)
{
    static constexpr float steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;
    static constexpr float filament_radius = DEFAULT_NOMINAL_FILAMENT_DIA / 2;
    static constexpr float step_multiplier = steps_per_unit[3] / (filament_radius * filament_radius * PI);
    uint32_t steps = uL * step_multiplier;
    return add_raw_extruder_steps(index, steps);
}
Result Controller::start_extruding(Index index)
{
    Packet packet(index, Command::SYRINGEPUMP_START);
    const auto result = send(packet, bus);
    if (result == Result::OK)
        set_extruder_state(index, true);
    return result;
}
Result Controller::stop_extruding(Index index)
{
    set_extruder_state(index, false);
    return Result::OK;
}
Result Controller::add_raw_extruder_steps(Index index, int32_t steps)
{
    Packet packet(index, Command::SYRINGEPUMP_DEBUG_ADD_STEPS, &steps, sizeof(steps));
    return send(packet, bus);
}

Result Controller::set_valve_speed(Index index, feedRate_t feedrate) {}
Response Controller::get_valve_speed(Index index) {}
Result Controller::set_valve_stallguard_threshold(Index index, uint8_t threshold) {}
Response Controller::get_valve_stallguard_threshold(Index index) {}
Result Controller::set_valve_microsteps(Index index, uint8_t microsteps) {}
Response Controller::get_valve_microsteps(Index index) {}
Result Controller::set_valve_rms_current(Index index, uint16_t mA)
{
    Packet packet(index, Command::ERROR, &mA, sizeof(mA));
    return send(packet, bus);
}
Response Controller::get_valve_rms_current(Index index)
{ // TODO: UNIMPLEMENTED
    Packet packet(index, Command::ERROR);
    return send_and_receive(packet, bus);
}
Result Controller::set_valve_hold_current(Index index, uint16_t mA)
{ // TODO: UNIMPLEMENTED
    Packet packet(index, Command::ERROR, &mA, sizeof(mA));
    return send(packet, bus);
}
Result Controller::home_slider_valve(Index index, SliderDirection dir)
{
    Packet packet(index, Command::SLIDER_MOVE_TO_HOME_POSITION, &dir, sizeof(dir));
    auto res = send(packet, bus);
    if (res == Result::OK) {
        ph_states[static_cast<uint8_t>(index)].slider_is_homed = true;
    }
    return res;
}
Result Controller::move_slider_valve(Index index, int32_t abs_steps)
{
    auto& state = ph_states[static_cast<uint8_t>(index)];
    int32_t rel_steps = abs_steps - state.slider_pos;
    Packet packet(index, Command::DEBUG_ADD_SLIDER_STEPS, &rel_steps, sizeof(rel_steps));
    auto result = send(packet, bus);
    if (result == Result::OK) {
        state.slider_pos = abs_steps;
    }
    return result;
}
Response Controller::get_uuid(Index index)
{
    Packet packet(index, Command::GET_UNIQUE_ID);
    return send_and_receive(packet, bus);
}
Response Controller::get_status(Index index)
{
    Packet packet(index, Command::GET_STATUS);
    return send_and_receive(packet, bus);
}

void Controller::stop_active_extrudes()
{
    for (size_t i = 0; i < EXTRUDERS; ++i) {
        if (ph_states[i].is_currently_extruding)
            stop_extruding(static_cast<printhead::Index>(i));
    }
}