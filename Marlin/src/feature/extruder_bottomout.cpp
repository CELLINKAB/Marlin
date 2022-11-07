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
    
static bool slider_valve_homed = false;

void bottomout_extruder(pin_t extruder_stop_pin)
{
    // lazy initialization to ensure good ordering with global systems.
    // static ensures this is only called on first run.
    static auto init_ [[maybe_unused]] = [] {
        tmc_enable_stallguard(stepperE0); 
    SET_INPUT_PULLUP(E0_STOP_PIN);
    tmc_enable_stallguard(stepperE1);
    SET_INPUT_PULLUP(E1_STOP_PIN);
        // TERN_(EXTRUDERS > 0, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(0));
        // TERN_(EXTRUDERS > 1, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(1));
        // TERN_(EXTRUDERS > 2, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(2));
        // TERN_(EXTRUDERS > 3, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(3));
        // TERN_(EXTRUDERS > 4, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(4));
        // TERN_(EXTRUDERS > 5, SETUP_EXTRUDER_BOTTOMOUT_STEPPER(5));
        return 0;
    }();
    callback_function_t bottomout_isr{[extruder_stop_pin] {
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
    planner.set_e_position_mm(0.0f);
    set_current_from_steppers_for_axis(AxisEnum::E_AXIS);
    sync_plan_position();
}

// constexpr pin_t get_extruder_stop_pin_from_index(int8_t extruder_index)
// {
//     switch (extruder_index) {
//         case 0: TERN_(E0_STOP_PIN, return E0_STOP_PIN);
//         case 1: TERN_(E1_STOP_PIN, return E1_STOP_PIN);
//         case 2: TERN_(E2_STOP_PIN, return E2_STOP_PIN);
//         case 3: TERN_(E3_STOP_PIN, return E3_STOP_PIN);
//         case 4: TERN_(E4_STOP_PIN, return E4_STOP_PIN);
//         case 5: TERN_(E5_STOP_PIN, return E5_STOP_PIN);
//         case 6: TERN_(E6_STOP_PIN, return E6_STOP_PIN);
//         default: return 0;
//     };
// }

/**
 * @brief Home extruder
 * 
 */
void GcodeSuite::G511()
{
    static bool homed = false;
    if (parser.seen_test('O') && homed) return;
    int8_t extruder_index = get_target_extruder_from_command();
    if (extruder_index == -1) return;
    bottomout_extruder(E0_STOP_PIN);
    homed = true;
}

/**
 * @brief Home slider valve
 * 
 */
void GcodeSuite::G512()
{
    if (parser.seen_test('O') && slider_valve_homed) return;
    const auto pre_command_extruder = active_extruder;
    active_extruder = 1; // hard coded slider valve as extruder for now
    bottomout_extruder(E1_STOP_PIN);
    active_extruder = pre_command_extruder;
    slider_valve_homed = true;
}

/**
 * @brief Slider valve move
 * 
 */
void GcodeSuite::G513()
{
    if (!slider_valve_homed) return;
    if (!parser.seenval('P')) return;
    const float position = parser.value_float();
    planner.synchronize();
    const auto pre_command_relative_mode = axis_relative;
    set_e_absolute();

    auto new_pos = current_position.copy();
    new_pos.e = position;
    planner.buffer_line(new_pos, 0.5, 1);
    planner.synchronize();

    // make sure relative mode settings weren't overwritten from user perspective
    axis_relative = pre_command_relative_mode;
}

#elif ENABLED(CHANTARELLE_SUPPORT) 

#include "../gcode/gcode.h"
#include "guppi_printhead/chantarelle.h"



#endif // HAS_E_BOTTOMOUT || CHANTARELLE_SUPPORT