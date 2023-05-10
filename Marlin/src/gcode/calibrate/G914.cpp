

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
        SERIAL_ECHO(seconds_until_start);
        for (size_t i = 0; i < 4; ++i) {
            safe_delay(250);
            SERIAL_CHAR('.');
        }
    }
    SERIAL_EOL();

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

    feedRate_t x_optimal_feedrate = parser.feedrateval('F', 5.0f);
    do_blocking_move_to_x(current_position.x - x_optimal_feedrate, x_optimal_feedrate);
    current_position.x += (x_optimal_feedrate * 2);
    planner.buffer_segment(current_position, x_optimal_feedrate);
    safe_delay(450);
    std::array<int32_t, 50> x_sg_samples{};
    for (auto& sample : x_sg_samples)
        sample = poll_sg_val(AxisEnum::X_AXIS);
    auto x_cur = stepperX.rms_current();

    float x_avg = static_cast<float>(std::accumulate(x_sg_samples.cbegin(), x_sg_samples.cend(), 0))
                  / static_cast<float>(x_sg_samples.size());
    auto x_min_max = std::minmax_element(x_sg_samples.cbegin(), x_sg_samples.cend());

    for (auto& sample : x_sg_samples)
        sample = (sample - x_avg) * (sample - x_avg);

    float x_var = static_cast<float>(std::accumulate(x_sg_samples.cbegin(), x_sg_samples.cend(), 0))
                  / static_cast<float>(x_sg_samples.size());
    float x_sd = sqrt(x_var);

    planner.synchronize();
    do_blocking_move_to_x(current_position.x - x_optimal_feedrate, x_optimal_feedrate);

    SERIAL_ECHOLNPGM("X feedrate: ",
                     x_optimal_feedrate,
                     ", current: ",
                     x_cur,
                     ", avg: ",
                     x_avg,
                     ", var: ",
                     x_var,
                     ", sd: ",
                     x_sd,
                     ", min: ",
                     *(x_min_max.first),
                     ", max: ",
                     *(x_min_max.second));

    SERIAL_ECHOLNPGM("Recommended M914 val: ", static_cast<uint8_t>((x_avg - (4.0f * x_sd)) / 2.0f));
}

#endif