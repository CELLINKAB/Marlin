// copyright Cellink 2022 GPL-v3

#include "../inc/MarlinConfig.h"

#define EXTRUDER_BOTTOMOUT // TODO: add to config
#if ENABLED(EXTRUDER_BOTTOMOUT)
#include "../gcode/gcode.h"
#include "../module/planner.h"
#include "../module/stepper/trinamic.h"
#include "../module/motion.h"

#include "interval_reporter.h"

void GcodeSuite::G511()
{
    // const uint8_t extruder = parser.byteval('E', 0xff);
    static auto init_ = []
    {
        tmc_enable_stallguard(stepperE0);
        SET_INPUT_PULLUP(E0_STOP_PIN);
        return 0;
    }();

    constexpr static feedRate_t feedrate = E_BOTTOMOUT_FEEDRATE;
    current_position.e -= E_BOTTOMOUT_BACKOFF;
    planner.buffer_segment(current_position, feedrate);

    planner.synchronize();

    static auto bottomout_isr = callback_function_t{[]
                                                    {
                                                        CRITICAL_SECTION_START();
                                                        planner.quick_stop();
                                                        CRITICAL_SECTION_END();
                                                        detachInterrupt(E0_STOP_PIN);
                                                    }};
    

    auto end_position = current_position.copy();
    end_position.e += E_BOTTOMOUT_MAX_DISTANCE;
    planner.buffer_segment(end_position, feedrate);
    attachInterrupt(
        E0_STOP_PIN,
        bottomout_isr,
        HIGH);
    planner.synchronize();
    set_current_from_steppers_for_axis(AxisEnum::E0_AXIS);
    sync_plan_position();
}
#endif