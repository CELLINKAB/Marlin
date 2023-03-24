

#include "request.h"

#include "../../MarlinCore.h"

using namespace printhead;

millis_t printhead::last_send = 0;

Result printhead::unsafe_send(const void* data, const size_t size, HardwareSerial& serial)
{
    OUT_WRITE(CHANT_RTS_PIN, HIGH);
    size_t sent = serial.write(static_cast<const uint8_t*>(data), size);
    serial.flush();
    WRITE(CHANT_RTS_PIN, LOW);
    if (sent != size)
        return Result::BUSY;
    return Result::OK;
}

void printhead::flush_rx(HardwareSerial& serial)
{
    while (serial.available())
        std::ignore = serial.read();
}

void Controller::tool_change(uint8_t tool_index)
{
    Index index = static_cast<Index>(tool_index);
    set_volume_per_fullstep(index, PL_PER_FULL_STEP);
    set_step_volume(index, PL_STEP_VOLUME);
}

void Controller::init()
{
    constexpr static unsigned CHANT_BAUDRATE = 115200;
    bus.begin(CHANT_BAUDRATE);
    tool_change(0);
}

void Controller::update(uint8_t tool_index)
{
    static millis_t next_update = 0;
    if (millis() < next_update)
        return;
    Index index = static_cast<Index>(tool_index);
    auto res = get_status(index, false);
    if (res.result == Result::OK)
        ph_states[tool_index].status = res.packet.payload;
    next_update = millis() + 500;
}

bool Controller::extruder_busy()
{
    return std::any_of(ph_states.cbegin(), ph_states.cend(), [](const PrintheadState& state) {
        return (state.status.is_stepping || state.status.is_homing);
    });
}

bool Controller::extruder_busy(Index index)
{
    const auto state = ph_states[static_cast<uint8_t>(index)];
    return state.status.is_stepping || state.status.is_homing;
}

bool Controller::slider_busy()
{
    return std::any_of(ph_states.cbegin(), ph_states.cend(), [](const PrintheadState& state) {
        return (state.status.slider_is_stepping);
    });
}

bool Controller::slider_busy(Index index)
{
    const auto state = ph_states[static_cast<uint8_t>(index)];
    return state.status.slider_is_stepping;
}

Result Controller::set_temperature(Index index, celsius_t temperature)
{
    uint16_t chant_temp = temperature + 30'000;
    Packet request(index, Command::SET_TEMP, chant_temp);
    return send(request, bus);
}

celsius_float_t Controller::get_temperature(Index index)
{
    static std::array<millis_t, EXTRUDERS> last_check_times;
    static std::array<celsius_float_t, EXTRUDERS> last_temps;
    const auto [oldest_it, newest_it] = std::minmax_element(last_check_times.cbegin(),
                                                            last_check_times.cend());
    if (index == Index::All || index == Index::None)
        return 0.0f;
    size_t index_num = static_cast<size_t>(index);
    if (millis() < *newest_it + 1000)
        return last_temps[index_num];
    size_t oldest_index_num = oldest_it - last_check_times.cbegin();
    Packet request(static_cast<Index>(oldest_index_num), Command::GET_MEASURED_TEMP);
    auto res = send_and_receive<celsius_t>(request, bus, false);
    last_check_times[oldest_index_num] = millis();
    if (res.result != Result::OK || res.packet.payload_size < 2)
        return 0.0f;
    last_temps[oldest_index_num] = (res.packet.payload - 30'000) / 100.0f;
    return last_temps[index_num];
}

Response<void> Controller::get_info(Index index)
{
    Packet packet(index, Command::GET_DEVICE_INFO);
    return send_and_receive<void>(packet, bus); // TODO: parse response into type
}

Response<void> Controller::get_fw_version(Index index)
{
    Packet packet(index, Command::GET_SW_VERSION);
    return send_and_receive<void>(packet, bus); //TODO: parse result into relevant type
}

Result Controller::set_pid(Index index, float p, float i, float d)
{
    uint16_t p_ = static_cast<uint16_t>(p * 100);
    uint16_t i_ = static_cast<uint16_t>(i * 100);
    uint16_t d_ = static_cast<uint16_t>(d * 100);

    Packet packet(index, Command::SET_PID, std::array{p_, i_, d_});
    return send(packet, bus);
}

Response<std::array<uint16_t, 3>> Controller::get_pid(Index index)
{
    Packet packet(index, Command::GET_PID);
    return send_and_receive<std::array<uint16_t, 3>>(packet,
                                                     bus); // TODO: parse incoming response and return payload values
}

auto Controller::set_fan_speed(Index index, FanSpeeds fan_speeds) -> Result
{
    Packet packet(index, Command::DEBUG_SET_FAN_PWM, fan_speeds);
    auto res = send(packet, bus);
    // if (res == Result::OK) // TODO: handle printhead state
    return res;
}

auto Controller::get_fan_speed(Index index) -> Response<FanSpeeds>
{
    Packet packet(index, Command::DEBUG_GET_FAN_PWM);
    return send_and_receive<FanSpeeds>(packet, bus); // TODO: handle printhead state, get TACH from chant
}

auto Controller::set_tem_debug(Index index, TemTemps tem_pwms) -> Result
{
    Packet packet(index, Command::DEBUG_SET_TEM_PWM, tem_pwms);
    return send(packet, bus);
}
auto Controller::get_tem_debug(Index index) -> Response<TemTemps>
{
    Packet packet(index, Command::DEBUG_GET_TEM_PWM);
    return send_and_receive<TemTemps>(packet, bus);
}

Result Controller::set_extrusion_speed(Index index, uint32_t feedrate_pl_s)
{
    Packet packet(index, Command::SET_EXTRUSION_SPEED, feedrate_pl_s);
    return send_and_receive<uint32_t>(packet, bus).result;
}

Response<uint32_t> Controller::get_extrusion_speed(Index index)
{
    Packet packet(index, Command::GET_EXTRUSION_SPEED);
    return send_and_receive<uint32_t>(packet, bus);
}

Result Controller::set_extruder_stallguard_threshold(Index index, uint8_t threshold)
{
    Packet packet(index, Command::SYRINGEPUMP_SET_ESTOP_THRESH, threshold);
    return send(packet, bus);
}

Response<uint8_t> Controller::get_extruder_stallguard_threshold(Index index)
{
    Packet packet(index, Command::SYRINGEPUMP_GET_ESTOP_THRESH);
    return send_and_receive<uint8_t>(packet, bus);
}

Result Controller::set_extruder_microsteps(Index index, uint8_t microsteps)
{
    Packet packet(index, Command::SET_MICROSTEP, microsteps);
    return send_and_receive<uint8_t>(packet, bus).result;
}

Response<uint8_t> Controller::get_extruder_microsteps(Index index)
{
    Packet packet(index, Command::GET_MICROSTEP);
    return send_and_receive<uint8_t>(packet, bus);
}

Result Controller::set_extruder_rms_current(Index index, uint16_t mA)
{
    return send(Packet{index, Command::DEBUG_SET_MOTOR_CURRENT, mA}, bus);
}

Result Controller::home_extruder(Index index, ExtruderDirection direction)
{
    Packet packet(index, Command::MOVE_TO_HOME_POSITION, direction);
    auto res = send(packet, bus);

    if (res == Result::OK) {
        auto& state = ph_states[static_cast<uint8_t>(index)];
        state.extruder_is_homed = true;
        state.status.is_homing = true;
    }
    return res;
}

Result Controller::set_extruder_direction(Index index, bool direction)
{
    return send_and_receive<uint8_t>(Packet(index,
                                            Command::SET_EXTRUSION_DIRECTION,
                                            static_cast<uint8_t>(direction)),
                                     bus)
        .result;
}

Result Controller::extruder_move(Index index, float uL)
{
    uint32_t steps = uL / MM_PER_MICRO_STEP;
    return add_raw_extruder_steps(index, steps);
}

Result Controller::start_extruding(Index index)
{
    UNUSED(index);
    return Result::OK;
}

Result Controller::stop_extruding(Index index)
{
    UNUSED(index);
    return Result::OK;
}

Result Controller::add_raw_extruder_steps(Index index, int32_t steps)
{
    Packet packet(index, Command::SYRINGEPUMP_DEBUG_ADD_STEPS, steps);
    auto res = send(packet, bus);
    if (res == Result::OK) {
        auto& state = ph_states[static_cast<uint8_t>(index)];
        state.status.is_stepping = true;
    }
    return res;
}

Result Controller::home_slider_valve(Index index, SliderDirection dir)
{
    auto& state = ph_states[static_cast<uint8_t>(index)];
    Packet packet(index, Command::SLIDER_MOVE_TO_HOME_POSITION, dir);
    auto res = send(packet, bus);
    if (res == Result::OK) {
        state.slider_is_homed = true;
        state.slider_pos = 0;
        state.status.slider_is_stepping = true;
    }
    return res;
}

Result Controller::move_slider_valve(Index index, int32_t abs_steps)
{
    auto& state = ph_states[static_cast<uint8_t>(index)];
    int32_t rel_steps = abs_steps - state.slider_pos;
    Packet packet(index, Command::DEBUG_ADD_SLIDER_STEPS, rel_steps);
    auto result = send(packet, bus);
    if (result == Result::OK) {
        state.slider_pos = abs_steps;
        state.status.slider_is_stepping = true;
    }
    return result;
}

Response<std::array<uint8_t, 12>> Controller::get_uuid(Index index)
{
    Packet packet(index, Command::GET_UNIQUE_ID);
    return send_and_receive<std::array<uint8_t, 12>>(packet, bus);
}

Response<Status> Controller::get_status(Index index, bool debug)
{
    Packet packet(index, Command::GET_STATUS);
    auto res = send_and_receive<uint16_t>(packet, bus, debug);
    return Response<Status>{Packet<Status>(res.packet.ph_index,
                                           res.packet.command,
                                           static_cast<Status>(res.packet.payload)),
                            res.result};
}

void Controller::stop_active_extrudes()
{
    for (size_t i = 0; i < EXTRUDERS; ++i) {
        if (ph_states[i].status.is_stepping)
            stop_extruding(static_cast<printhead::Index>(i));
    }
}

Result Controller::set_volume_per_fullstep(Index index, uint32_t picoliters)
{
    Packet packet(index, Command::SYRINGEPUMP_SET_FULLSTEP_VOLUME, picoliters);
    return send_and_receive<uint32_t>(packet, bus).result;
}

Result Controller::set_step_volume(Index index, uint32_t picoliters)
{
    Packet packet(index, Command::SET_STEP_VOLUME, picoliters);
    return send_and_receive<uint32_t>(packet, bus).result;
}

Response<uint32_t> Controller::get_step_volume(Index index)
{
    Packet packet(index, Command::GET_STEP_VOLUME);
    return send_and_receive<uint32_t>(packet, bus);
}
