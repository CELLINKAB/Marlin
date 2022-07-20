

#include "../inc/MarlinConfig.h"

#if ENABLED(STEPPER_RETRACTING_PROBE)

#    include "../gcode/gcode.h"
#    include "../gcode/parser.h"

#    include "stepper_retracting_probe.h"

StepperRetractingProbe stepper_probe;

void GcodeSuite::M1029()
{
    if (!parser.seen_any()) {
        stepper_probe.report_config(false);
        return;
    }
    StepperRetractingProbe::Config new_conf = stepper_probe.get_config();
    if (parser.seen('T'))
        new_conf.stall_threshold = parser.value_byte();
    if (parser.seen('C'))
        new_conf.stepper_current = parser.value_int();
    if (parser.seen('D'))
        new_conf.deploy_velocity = parser.value_int();
    if (parser.seen('S'))
        new_conf.stow_velocity = parser.value_int();
    if (parser.seen('B'))
        new_conf.minimum_retract_time = parser.value_int();
    stepper_probe.set_config(new_conf);
}

#endif