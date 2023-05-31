

#include "../gcode.h"

#if ENABLED(SENSORLESS_HOMING)

#    include "../../module/stepper/trinamic.h"
#    include "../parser.h"

#    include <array>
#    include <numeric>

void GcodeSuite::G914()
{
    set_all_unhomed();
    // TODO: disable steppers here so you can move manually

    SERIAL_ECHOLN("Manually move the printbed to the center of the movable area");
    for (size_t seconds_until_start = 5; seconds_until_start > 0; --seconds_until_start) {
        SERIAL_ECHOLN(seconds_until_start);
        safe_delay(1000);
    }

    auto poll_sg_val = [](AxisEnum axis) -> int32_t {
        static millis_t last_return_time = 0;
        // spin until next result needed
        while (millis() <= last_return_time + 20)
            idle_no_sleep();
        last_return_time = millis();
        switch (axis) {
        case AxisEnum::X_AXIS:
            return stepperX.SG_RESULT();
        case AxisEnum::Y_AXIS:
            return stepperY.SG_RESULT();
        case AxisEnum::Z_AXIS:
            return stepperZ.SG_RESULT();
        default:
            return 0;
        }
    };

    feedRate_t x_optimal_feedrate = parser.feedrateval('F', 30.0f);
    auto x_cur = parser.ushortval('C', 600);

    stepperX.rms_current(x_cur);
    do_blocking_move_to_x(current_position.x - x_optimal_feedrate, x_optimal_feedrate);
    SERIAL_ECHOLNPGM("feedrate: ", x_optimal_feedrate, ", current: ", x_cur);
    current_position.x += (x_optimal_feedrate * 2);
    planner.buffer_segment(current_position, x_optimal_feedrate);
    while (planner.busy()) {
        SERIAL_ECHOLNPGM("SG:", poll_sg_val(AxisEnum::X_AXIS));
    }


    planner.synchronize();
    do_blocking_move_to_x(current_position.x - x_optimal_feedrate, x_optimal_feedrate);

}

#endif