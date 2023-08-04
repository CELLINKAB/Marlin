

#include "../inc/MarlinConfig.h"

#if ENABLED(STEPPER_RETRACTING_PROBE)

#    include "../gcode/gcode.h"
#    include "../gcode/parser.h"
#    include "../module/endstops.h"

#    include "stepper_retracting_probe.h"

void StepperRetractingProbe::unstick(int32_t velocity){
    int32_t unstick_velocity = (velocity >= 0) ? 8000 : -8000;
    auto unstick_move = [=](millis_t move_time){
        stepper().raw_move(unstick_velocity);
        delay(move_time);
        stepper().stop();
        safe_delay(1);
    };
    for (size_t i = 0; i < 5; ++i) {
        unstick_move(10);
    }
    unstick_move(50);
}

void StepperRetractingProbe::deploy()
{
    switch (state) {
    case ProbeState::Deployed:
        break;
    case ProbeState::Unknown:
        stepper().raw_move(config.stow_velocity);
        safe_delay(200);
        stepper().stop();
        delay(10);
        [[fallthrough]];
    case ProbeState::Stowed:
        // FIXME: re-enable stallguard move
        // stepper().blocking_move_until_stall(config.deploy_velocity, config.minimum_retract_time * 2);
        unstick(config.deploy_velocity); 
        stepper().raw_move(config.deploy_velocity);
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
        state = ProbeState::Deployed;
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
    case ProbeState::Stowed:
        break;
    case ProbeState::Unknown:
        deploy();
        delay(10);
        [[fallthrough]];
    case ProbeState::Deployed:
        unstick(config.stow_velocity);
        stepper().raw_move(config.stow_velocity);
        safe_delay(config.minimum_retract_time);
        stepper().stop();
        state = ProbeState::Stowed;
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
                           " M",
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
                           "\nBackoff time: ",
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