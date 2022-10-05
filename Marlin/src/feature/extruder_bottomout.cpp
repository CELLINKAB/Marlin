// copyright Cellink 2022 GPL-v3

#include "../inc/MarlinConfig.h"

#if HAS_E_BOTTOMOUT
#    include "../gcode/gcode.h"
#    include "../module/motion.h"
#    include "../module/planner.h"
#    include "../module/stepper/trinamic.h"

#    include "interval_reporter.h"

#define SETUP_EXTRUDER_BOTTOMOUT_STEPPER(index) \
    tmc_enable_stallguard(stepperE##index); \
    SET_INPUT_PULLUP(E##index##_STOP_PIN)
    

void bottomout_extruder(pin_t extruder_stop_pin)
{
    // lazy initialization to ensure good ordering with global systems.
    // static ensures this is only called on first run.
    static auto init_ [[maybe_unused]] = [] {
        TERN_(EXTRUDERS > 0, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(0));
        TERN_(EXTRUDERS > 1, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(1));
        TERN_(EXTRUDERS > 2, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(2));
        TERN_(EXTRUDERS > 3, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(3));
        TERN_(EXTRUDERS > 4, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(4));
        TERN_(EXTRUDERS > 5, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(5));
        return 0;
    }();
    static callback_function_t bottomout_isr{[extruder_stop_pin] {
        // critical section necessary as stepper ISR running during
        // planner.quick_stop() creates a race condition for clearing
        // the block buffer causing an infinite "busy" loop.
        CRITICAL_SECTION_START();
        planner.quick_stop();
        CRITICAL_SECTION_END();
        detachInterrupt(extruder_stop_pin);
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
    attachInterrupt(extruder_stop_pin, bottomout_isr, HIGH);

    planner.synchronize(); // spins until move completed or bottomout_isr clears buffer

    // cleanup needed to ensure future moves work as expected
    set_current_from_steppers_for_axis(AxisEnum::E_AXIS);
    sync_plan_position();
}

constexpr pin_t get_extruder_stop_pin_from_index(int8_t extruder_index)
{
    switch (extruder_index) {
        case 0: TERN_(defined(E0_STOP_PIN), return E0_STOP_PIN);
        case 1: /*TERN_(defined(E1_STOP_PIN),*/ return E1_STOP_PIN;
        case 2: TERN_(defined(E2_STOP_PIN), return E2_STOP_PIN);
        case 3: TERN_(defined(E3_STOP_PIN), return E3_STOP_PIN);
        case 4: TERN_(defined(E4_STOP_PIN), return E4_STOP_PIN);
        case 5: TERN_(defined(E5_STOP_PIN), return E5_STOP_PIN);
        case 6: TERN_(defined(E6_STOP_PIN), return E6_STOP_PIN);
        default: return 0;
    };
}

void GcodeSuite::G511()
{
    int8_t extruder_index = get_target_extruder_from_command();
    if (extruder_index == -1) return;
    bottomout_extruder(get_extruder_stop_pin_from_index(extruder_index));
}
#endif // HAS_E_BOTTOMOUT