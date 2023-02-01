

#include "../inc/MarlinConfig.h"

#if ENABLED(STEPPER_RETRACTING_PROBE)

#    include "../gcode/gcode.h"
#    include "../gcode/parser.h"

#    include "stepper_retracting_probe.h"



void StepperRetractingProbe::deploy()
{
    switch (state) {
    case ProbeState::Deployed:
        [[fallthrough]];
    case ProbeState::Unknown:
        stepper().raw_move(config.stow_velocity);
        safe_delay(200);
        stepper().stop();
        [[fallthrough]];
    case ProbeState::Stowed:
    // FIXME: re-enable stallguard move
        // stepper().blocking_move_until_stall(config.deploy_velocity, config.minimum_retract_time * 2);
        stepper().raw_move(config.deploy_velocity);
        safe_delay(config.minimum_retract_time * 2);
        stepper().stop();
        state = ProbeState::Deployed;
    }
}

void StepperRetractingProbe::stow()
{
    if (state != ProbeState::Deployed) {
        deploy();
    }
    stepper().raw_move(config.stow_velocity);
    safe_delay(config.minimum_retract_time);
    stepper().stop();
    state = ProbeState::Stowed;
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