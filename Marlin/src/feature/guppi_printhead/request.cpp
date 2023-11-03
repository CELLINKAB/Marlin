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

#include "../../MarlinCore.h"

#if ENABLED(CHANTARELLE_SUPPORT)

#    include "../../module/motion.h"

#    include "request.h"

using namespace printhead;

millis_t printhead::last_serial_activity = 0;

uint32_t printhead::avg_latency_us = 0;
uint32_t printhead::min_latency_us = std::numeric_limits<uint32_t>::max();
uint32_t printhead::max_latency_us = 0;
uint32_t printhead::request_start_us = 0;


void printhead::flush_rx(HardwareSerial& serial)
{
    while (serial.read() >= 0)
        ;
}

void Controller::tool_change(uint8_t tool_index)
{
    Index index = static_cast<Index>(tool_index);
    set_volume_per_fullstep(index, PL_PER_FULL_STEP);
    set_step_volume(index, PL_STEP_VOLUME);
}

void Controller::init()
{
    constexpr static unsigned CHANT_DEFAULT_BAUDRATE = 115200;
    constexpr static unsigned CHANT_DEFAULT_TIMEOUT = 200;
    #if PINS_EXIST(CHANT_DE, CHANT_RE)
      OUT_WRITE(CHANT_DE_PIN, HIGH);
      OUT_WRITE(CHANT_RE_PIN, HIGH);
    #endif
    bus.begin(CHANT_DEFAULT_BAUDRATE);
    bus.setTimeout(CHANT_DEFAULT_TIMEOUT);
    tool_change(0);
}

//
// Update handling
//

enum class UpdateState {
    ENCODERS,
    STATUS_0,
    TEMPERATURE_0,
    STATUS_1,
    TEMPERATURE_1,
    STATUS_2,
    TEMPERATURE_2,
    STATUS_3,
    NUM_STATES
};

[[nodiscard]] constexpr UpdateState next_update_state(UpdateState current_state)
{
    return static_cast<UpdateState>((static_cast<uint>(current_state) + 1)
                                    % static_cast<uint>(UpdateState::NUM_STATES));
}

constexpr auto update_state_to_str(UpdateState current_state)
{
    switch (current_state) {
    case UpdateState::ENCODERS:
        return "ENCODERS";
    case UpdateState::TEMPERATURE_0:
        [[fallthrough]];
    case UpdateState::TEMPERATURE_1:
        [[fallthrough]];
    case UpdateState::TEMPERATURE_2:
        return "TEMPERATURE";
    case UpdateState::STATUS_0:
        [[fallthrough]];
    case UpdateState::STATUS_1:
        [[fallthrough]];
    case UpdateState::STATUS_2:
        [[fallthrough]];
    case UpdateState::STATUS_3:
        return "STATUS";
    case UpdateState::NUM_STATES:
        break;
    }
    return "UNREACHABLE";
}

void Controller::update(millis_t now)
{
    static millis_t next_update = 0;
    static UpdateState update_state = UpdateState::ENCODERS;

    static auto retry = [](PrintheadState& ph_state) {
        static constexpr size_t MAX_RETRIES = 3;
        static size_t retries = 0;
        if (++retries > MAX_RETRIES) {
            ph_state.status = {};
            update_state = next_update_state(update_state);
            retries = 0;
            if (DEBUGGING(ERRORS))
                SERIAL_ECHOLNPGM("Retries exceeded for updating ", update_state_to_str(update_state));
        }
    };

    static auto update_temperature = [this](uint8_t tool_index) {
        auto& state = ph_states[tool_index];
        const Index index = static_cast<Index>(tool_index);
        const auto temp_res = get_temperature(index, false);
        if (temp_res.result == Result::OK) {
            // TODO: change back when get_temp is fixed on printhead
            state.raw_temperature = temp_res.packet.payload;
            update_state = next_update_state(update_state);
        } else
            retry(state);
    };

    static auto update_status = [this](uint8_t tool_index) {
        auto& state = ph_states[tool_index];
        const Index index = static_cast<Index>(tool_index);
        const auto status_res = get_status(index, false);
        if (status_res.result == Result::OK) {
            if (DEBUGGING(LEVELING)) {
                if (state.status.is_homing && !status_res.packet.payload.is_homing)
                    SERIAL_ECHO_MSG("extruder finished homing");
                if (state.status.slider_is_stepping && !status_res.packet.payload.slider_is_stepping)
                    SERIAL_ECHO_MSG("slider valve move finished");
            }
            state.status = status_res.packet.payload;
            update_state = next_update_state(update_state);
        } else
            retry(state);
    };

    if (now < next_update || disable_background_updates)
        return;

    switch (update_state) {
    case UpdateState::ENCODERS: {
        const auto encoder_res = debug_get_encoders(false);
        if (encoder_res.result == Result::OK) {
            ph_states[0].extruder_encoder = get_encoder_state(encoder_res.packet.payload,
                                                              EncoderIndex::ExtruderOne);
            ph_states[1].extruder_encoder = get_encoder_state(encoder_res.packet.payload,
                                                              EncoderIndex::ExtruderTwo);
            ph_states[2].extruder_encoder = get_encoder_state(encoder_res.packet.payload,
                                                              EncoderIndex::ExtruderThree);
            ph_states[0].slider_encoder = get_encoder_state(encoder_res.packet.payload,
                                                            EncoderIndex::SliderOne);
            ph_states[1].slider_encoder = get_encoder_state(encoder_res.packet.payload,
                                                            EncoderIndex::SliderTwo);
            ph_states[2].slider_encoder = get_encoder_state(encoder_res.packet.payload,
                                                            EncoderIndex::SliderThree);
            update_state = next_update_state(update_state);
        } else {
            PrintheadState ignore;
            retry(ignore);
        }

        break;
    }

    case UpdateState::TEMPERATURE_0:
        update_temperature(0);
        break;
    case UpdateState::TEMPERATURE_1:
        update_temperature(1);
        break;
    case UpdateState::TEMPERATURE_2:
        update_temperature(2);
        break;

    case UpdateState::STATUS_0:
        [[fallthrough]];
    case UpdateState::STATUS_1:
        [[fallthrough]];
    case UpdateState::STATUS_2:
        update_status(active_extruder);
        break;
    case UpdateState::STATUS_3: {
        static uint8_t tool_index = 0;
        update_status(tool_index);
        ++tool_index;
        if (tool_index >= EXTRUDERS) {
            tool_index = 0;
        } else {
            update_state = UpdateState::STATUS_3;
        }
    } break;

    default:
        update_state = UpdateState::ENCODERS;
        break;
    }

    static constexpr millis_t UPDATE_INTERVAL = SEC_TO_MS(1) / 8; // 125ms
    next_update = now + UPDATE_INTERVAL;
}

void Controller::report_states()
{
    {
#    define PRINT_FIELD(field) SERIAL_ECHOPGM(#field ": ", state.field, ", ");

        SERIAL_ECHOLN("Printheads: [");
        for (const auto& state : ph_states) {
            SERIAL_ECHO("State: { ");
            PRINT_FIELD(extruder_encoder);
            PRINT_FIELD(slider_pos);
            PRINT_FIELD(slider_encoder);
            SERIAL_ECHOLN("}");
        }
        SERIAL_ECHOLN("]");
    }
}

celsius_float_t Controller::get_latest_extruder_temp(Index index)
{
    const auto& state = ph_states[static_cast<uint8_t>(index)];
    return (state.raw_temperature - 30'000) / 100.0f;
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

void Controller::set_timeout(millis_t timeout)
{
    bus.setTimeout(timeout);
}

Response<uint16_t> Controller::set_temperature(Index index, celsius_float_t temperature)
{
    uint16_t chant_temp = static_cast<uint16_t>((temperature * 100.0f) + 30'000);
    Packet request(index, Command::SET_TEMP, chant_temp);
    return send_and_receive<uint16_t>(request, bus);
}

Response<uint16_t> Controller::get_temperature(Index index, bool debug)
{
    Packet request(index, Command::GET_MEASURED_TEMP);
    return send_and_receive<uint16_t>(request, bus, debug);
}

Response<void> Controller::get_info(Index index)
{
    Packet packet(index, Command::GET_DEVICE_INFO);
    return send_and_receive<void>(packet, bus); // TODO: parse response into type
}

Response<FirmwareVersion> Controller::get_fw_version(Index index)
{
    Packet packet(index, Command::GET_SW_VERSION);
    return send_and_receive<FirmwareVersion>(packet, bus);
}

Response<PIDParams> Controller::set_pid(Index index, float p, float i, float d)
{
    uint16_t p_ = static_cast<uint16_t>(p * 100);
    uint16_t i_ = static_cast<uint16_t>(i * 100);
    uint16_t d_ = static_cast<uint16_t>(d * 100);

    Packet packet(index, Command::SET_PID, std::array{p_, i_, d_});
    return send_and_receive<PIDParams>(packet, bus);
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
    return send_and_receive<FanSpeeds>(packet,
                                       bus); // TODO: handle printhead state, get TACH from chant
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

Response<EncoderStates> Controller::debug_get_encoders(bool debug)
{
    // Semantically it makes sense to use the ALL address here since
    // this currently is implemented by returning all encoders.
    // However, receiving an ALL address disables replies in Guppi.
    Packet packet(Index::One, Command::DEBUG_GET_ENCODERS);
    return send_and_receive<EncoderStates>(packet, bus, debug);
}

Result Controller::disable_heating(Index index)
{
    Packet packet(index, Command::DISABLE_TEMP_CONTROL);
    return send(packet, bus);
}

Response<TemTemps> Controller::debug_get_temperature(Index index, bool debug)
{
    Packet packet(index, Command::DEBUG_GET_TEMPERATURE);
    return send_and_receive<TemTemps>(packet, bus, debug);
}

#endif // CHANTARELLE_SUPPORT