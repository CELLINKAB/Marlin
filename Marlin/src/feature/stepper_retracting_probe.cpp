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

#include "../inc/MarlinConfig.h"

#if ENABLED(STEPPER_RETRACTING_PROBE)

#    include "../gcode/gcode.h"
#    include "../gcode/parser.h"
#    include "../module/endstops.h"

#    include "stepper_retracting_probe.h"

void StepperRetractingProbe::unstick(int32_t velocity)
{
    int32_t unstick_velocity = (velocity >= 0) ? 8000 : -8000;
    auto unstick_move = [this, unstick_velocity](int32_t velocity, millis_t move_time) {
        stepper().ramped_move(velocity);
        safe_delay(move_time);
        stepper().gentle_stop();
    };
    for (size_t i = 0; i < 3; ++i) {
        unstick_move(unstick_velocity, 10);
    }
}

void StepperRetractingProbe::backoff()
{
    start_move(config.stow_velocity);
    safe_delay(200);
    stepper().gentle_stop();
    safe_delay(10);
}

void StepperRetractingProbe::deploy()
{
    switch (state) {
    case State::Deployed:
        break;
    case State::Unknown:
        backoff();
        [[fallthrough]];
    case State::Stowed:
        // FIXME: re-enable stallguard move
        // stepper().blocking_move_until_stall(config.deploy_velocity, config.minimum_retract_time * 2);
        start_move(config.deploy_velocity);
        const millis_t minimum_deploy_time = static_cast<millis_t>(
            static_cast<float>(config.minimum_retract_time)
                * ABS(static_cast<float>(config.stow_velocity)
                      / static_cast<float>(config.deploy_velocity))
            + 1000.0f);
        const millis_t timeout = millis() + minimum_deploy_time;
        bool probe_hit = false;
        const auto was_enabled = endstops.z_probe_enabled;
        endstops.enable_z_probe();
        while (!(probe_hit = TEST(endstops.state(), Z_MIN_PROBE)) && millis() < timeout) {
            idle();
        }
        stepper().stop();
        state = State::Deployed;
        if (probe_hit) {
            stow();
        }
        endstops.enable_z_probe(was_enabled);
        break;
    }
}

void StepperRetractingProbe::stow()
{
    switch (state) {
    case State::Stowed:
        break;
    case State::Unknown:
        [[fallthrough]];
    case State::Deployed:
        start_move(config.stow_velocity);
        safe_delay(config.minimum_retract_time);
        stepper().stop();
        state = State::Stowed;
        break;
    }
}

void StepperRetractingProbe::report_config(bool for_replay) const
{
    if (for_replay) {
        SERIAL_ECHOLNPGM_P("M1029 T",
                           config.stall_threshold,
                           " C",
                           config.stepper_current,
                           " S",
                           config.stow_velocity,
                           " D",
                           config.deploy_velocity,
                           " B",
                           config.minimum_retract_time);
    } else {
        SERIAL_ECHOLNPGM_P("Stall threshold: ",
                           config.stall_threshold,
                           "\nStepper current: ",
                           config.stepper_current,
                           "\nStow velocity: ",
                           config.stow_velocity,
                           "\nDeploy velocity: ",
                           config.deploy_velocity,
                           "\nRetract time: ",
                           config.minimum_retract_time);
    }
}

void StepperRetractingProbe::stepper_init()
{
#    if PINS_EXIST(SP_SERIAL_TX, SP_SERIAL_RX)
    _stepper.emplace(SimpleTMCConfig(PROBE_SERIAL_ADDRESS,
                                     config.stall_threshold,
                                     config.stepper_current,
                                     SRP_STEPPER_RSENSE),
                     SP_SERIAL_RX_PIN,
                     SP_SERIAL_TX_PIN);
#    elif defined(SP_HARDWARE_SERIAL)
    _stepper.emplace(STMC(SimpleTMCConfig(PROBE_SERIAL_ADDRESS,
                                          config.stall_threshold,
                                          config.stepper_current,
                                          SRP_STEPPER_RSENSE),
                          &SP_HARDWARE_SERIAL));
#    else
#        error "need to define SP_SERIAL_TX/RX_PIN or SP_HARDWARE_SERIAL for stepper retracting probe"
#    endif
}

void StepperRetractingProbe::init()
{
    state = State::Unknown;
    init_done_time = millis() + config.minimum_retract_time;
    start_move(config.stow_velocity);
}

void StepperRetractingProbe::update(millis_t ms)
{
    if (state != State::Unknown || init_done_time == 0 || ms < init_done_time)
        return;

    // state is unknown, there's an init_done_time, and we're past it, so finish init
    stepper().stop();
    state = State::Stowed;
    init_done_time = 0;
}

StepperRetractingProbe stepper_probe;

void GcodeSuite::M1029()
{
    if (!parser.seen_any()) {
        stepper_probe.report_config(false);
        return;
    }
    if (parser.seen('R'))
        stepper_probe.reset_position();
    StepperRetractingProbe::Config new_conf = stepper_probe.get_config();
    if (parser.seen('T'))
        new_conf.stall_threshold = parser.value_int();
    if (parser.seen('C'))
        new_conf.stepper_current = parser.value_ushort();
    if (parser.seen('D'))
        new_conf.deploy_velocity = parser.value_long();
    if (parser.seen('S'))
        new_conf.stow_velocity = parser.value_long();
    if (parser.seen('B'))
        new_conf.minimum_retract_time = parser.value_ulong();
    stepper_probe.set_config(new_conf);
}

#endif