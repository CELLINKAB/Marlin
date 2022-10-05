// copyright Cellink 2022 GPL-v3

#include "../inc/MarlinConfig.h"

#if HAS_E_BOTTOMOUT
#    include "../gcode/gcode.h"
#    include "../module/motion.h"
#    include "../module/planner.h"
#    include "../module/stepper/trinamic.h"

#    include "interval_reporter.h"

void GcodeSuite::G511()
{
    // lazy initialization to ensure good ordering with global systems.
    // static ensures this is only called on first run.
    static auto init_ [[maybe_unused]] = [] {
        tmc_enable_stallguard(stepperE0);
        SET_INPUT_PULLUP(E0_STOP_PIN);
        return 0;
    }();
    static callback_function_t bottomout_isr{[] {
        // critical section necessary as stepper ISR running during
        // planner.quick_stop() creates a race condition for clearing
        // the block buffer causing an infinite "busy" loop.
        CRITICAL_SECTION_START();
        planner.quick_stop();
        CRITICAL_SECTION_END();
        detachInterrupt(E0_STOP_PIN);
    }};
    constexpr static feedRate_t feedrate = E_BOTTOMOUT_FEEDRATE;

    // backoff in case extruder is already bottomed out, increases reliability.
    current_position.e -= E_BOTTOMOUT_BACKOFF;
    planner.buffer_segment(current_position, feedrate);
    planner.synchronize();

    const auto end_position = [] {
        auto pos = current_position.copy();
        pos.e += E_BOTTOMOUT_MAX_DISTANCE;
        return pos;
    }();
    planner.buffer_segment(end_position, feedrate);
    attachInterrupt(E0_STOP_PIN, bottomout_isr, HIGH);

    planner.synchronize(); // spins until move completed or bottomout_isr clears buffer

    // cleanup needed to ensure future moves work as expected
    set_current_from_steppers_for_axis(AxisEnum::E0_AXIS);
    sync_plan_position();
}

void GcodeSuite::G512()
{
    // lazy initialization to ensure good ordering with global systems.
    // static ensures this is only called on first run.
    static auto init_ [[maybe_unused]] = [] {
        tmc_enable_stallguard(stepperE1);
        SET_INPUT_PULLUP(E1_STOP_PIN);
        return 0;
    }();
    static callback_function_t bottomout_isr{[] {
        // critical section necessary as stepper ISR running during
        // planner.quick_stop() creates a race condition for clearing
        // the block buffer causing an infinite "busy" loop.
        CRITICAL_SECTION_START();
        planner.quick_stop();
        CRITICAL_SECTION_END();
        detachInterrupt(E1_STOP_PIN);
    }};
    constexpr static feedRate_t feedrate = E_BOTTOMOUT_FEEDRATE;

    // backoff in case extruder is already bottomed out, increases reliability.
    current_position.e -= E_BOTTOMOUT_BACKOFF;
    planner.buffer_segment(current_position, feedrate);
    planner.synchronize();

    const auto end_position = [] {
        auto pos = current_position.copy();
        pos.e += E_BOTTOMOUT_MAX_DISTANCE;
        return pos;
    }();
    planner.buffer_segment(end_position, feedrate);
    attachInterrupt(E1_STOP_PIN, bottomout_isr, HIGH);

    planner.synchronize(); // spins until move completed or bottomout_isr clears buffer

    // cleanup needed to ensure future moves work as expected
    set_current_from_steppers_for_axis(AxisEnum::E1_AXIS);
    sync_plan_position();
}


#endif // HAS_E_BOTTOMOUT