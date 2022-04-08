// copyright Cellink 2022 GPL-v3

#include "../inc/MarlinConfig.h"

#define EXTRUDER_BOTTOMOUT // TODO: add to config
#if ENABLED(EXTRUDER_BOTTOMOUT)
#include "../gcode/gcode.h"
#include "../module/planner.h"
#include "../module/stepper/trinamic.h"

#include "interval_reporter.h"

void GcodeSuite::G511()
{
    // const uint8_t extruder = parser.byteval('E', 0xff);
    static auto init_ = []
    {
        tmc_enable_stallguard(stepperE0);
        SET_INPUT_PULLDOWN(E0_STOP_PIN);
        return 0;
    }();

    if (parser.boolval('S'))
    {
        static auto check_stop_pin = IntervalReporter{[]
                                                      { SERIAL_ECHOPGM("E_STOP:", READ(E0_STOP_PIN), ", SG_result:", stepperE0.SG_RESULT()); }};
        check_stop_pin.start();
        return;
    }

    if (READ(E0_STOP_PIN))
    {
        SERIAL_ERROR_MSG("motor already signalling stall");

        return;
    }

    bool stall_triggered = false;
    attachInterrupt(E0_STOP_PIN, callback_function_t{[&stall_triggered]()
                                                     {
                                                         stall_triggered = true;
                                                         planner.quick_stop();
                                                         sync_plan_position_e();
                                                        planner.synchronize();
                                                     }},
                    RISING);

    auto end_position = current_position.copy();
    end_position.e += 5000.0f;
    planner.buffer_segment(end_position, parser.feedrateval('F', 1.0f));
    while (!stall_triggered)
    {
        delay(100);
    }
    detachInterrupt(E0_STOP_PIN);
}
#endif