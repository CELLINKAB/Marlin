

#include "../gcode.h"

#if ENABLED(SENSORLESS_HOMING)

#    include "../../module/stepper.h"
#    include "../../module/stepper/trinamic.h"
#    include "../parser.h"

#    include <array>
#    include <numeric>

void tune_axis(AxisEnum axis, uint16_t cur, feedRate_t feedrate)
{
    static auto poll_sg_val = [](AxisEnum axis) -> int32_t {
        static millis_t last_return_time = 0;
        // spin until next result needed
        while (millis() <= last_return_time + 10)
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

    static auto set_axis_current = [](AxisEnum axis, uint16_t mA) {
        switch (axis) {
        case AxisEnum::X_AXIS:
            stepperX.rms_current(mA);
            TERN_(HAS_DUAL_X_STEPPERS, stepperX2.rms_current(mA));
            return stepperX.rms_current();
        case AxisEnum::Y_AXIS:
            stepperY.rms_current(mA);
            TERN_(HAS_DUAL_Y_STEPPERS, stepperY2.rms_current(mA));
            return stepperY.rms_current();
        case AxisEnum::Z_AXIS:
            stepperZ.rms_current(mA);
            // TODO: add multi Z support
            return stepperZ.rms_current();
        default:
            return static_cast<uint16_t>(0);
        }
    };

    static auto set_axis_sg_thresh = [](AxisEnum axis, uint16_t thresh) {
        switch (axis) {
        case AxisEnum::X_AXIS:
            stepperX.homing_threshold(thresh);
            TERN_(HAS_DUAL_X_STEPPERS, stepperX2.homing_threshold(thresh));
            break;
        case AxisEnum::Y_AXIS:
            stepperY.homing_threshold(thresh);
            TERN_(HAS_DUAL_Y_STEPPERS, stepperY2.homing_threshold(thresh));
            break;
        case AxisEnum::Z_AXIS:
            stepperZ.homing_threshold(thresh);
            // TODO: add multi Z support
            break;
        default:
            break;
        }
    };

    auto move_cur = set_axis_current(axis, cur);
    auto dir = home_dir(axis);
    current_position[axis] += ((feedrate / 2) * -dir);
    do_blocking_move_to(current_position, feedrate);
    SERIAL_ECHOLNPGM("feedrate: ", feedrate, ", current: ", cur);
    SERIAL_ECHO("Axis: ");
    SERIAL_CHAR(AXIS_CHAR(axis));
    SERIAL_EOL();

    static constexpr size_t MAX_SAMPLES = 64;
    std::array<uint16_t, MAX_SAMPLES> sg_samples;

    uint32_t sum = 0;

    current_position[axis] += (feedrate * dir);
    planner.buffer_segment(current_position, feedrate);
    safe_delay(200); // make sure we're into the move
    for (auto& sample : sg_samples) {
        sample = poll_sg_val(axis);
        sum += sample;
        if (DEBUGGING(LEVELING))
            SERIAL_ECHOLNPGM("SG:", sample);
    }

    uint32_t avg = sum / MAX_SAMPLES;
    float var = 0.0f;
    for (const auto sample : sg_samples) {
        var += powf(static_cast<float>(sample) - static_cast<float>(avg), 2.0f)
               / static_cast<float>(MAX_SAMPLES);
    }
    float sd = sqrt(var);

    auto [min_p, max_p] = std::minmax_element(sg_samples.cbegin(), sg_samples.cend());

    if (DEBUGGING(INFO))
        SERIAL_ECHOLNPGM("avg: ",
                         avg,
                         ", var: ",
                         var,
                         ", std: ",
                         sd,
                         "\nmin: ",
                         *min_p,
                         ", max: ",
                         *max_p,
                         ", range: ",
                         (*max_p - *min_p));

    uint16_t recommended_thresh = *min_p / 2;
    recommended_thresh /= 2;
    set_axis_sg_thresh(axis, recommended_thresh);
    SERIAL_ECHOLNPGM("SG_THRESHOLD: ", recommended_thresh);

    planner.synchronize();
    current_position[axis] += ((feedrate / 2) * -dir);
    do_blocking_move_to(current_position, feedrate);

    set_axis_current(axis, move_cur);
}

void GcodeSuite::G914()
{
    set_all_unhomed();

    SERIAL_ECHOLN("Manually move the printbed to the center of the movable area");
    for (size_t seconds_until_start = 5; seconds_until_start > 0; --seconds_until_start) {
        SERIAL_ECHOLN(seconds_until_start);
        safe_delay(1000);
    }

    static constexpr xyz_uint_t default_home_feedrate = HOMING_FEEDRATE_MM_M;
    feedRate_t feedrate = parser.feedrateval('F',
                                             MMM_TO_MMS(default_home_feedrate[AxisEnum::X_AXIS]));
    auto cur = parser.ushortval('C', X_CURRENT_HOME);

    if (!parser.seen_axis()) {
        tune_axis(AxisEnum::X_AXIS, cur, feedrate);
        tune_axis(AxisEnum::Y_AXIS, cur, feedrate);
        tune_axis(AxisEnum::Z_AXIS, cur, feedrate);
        return;
    }

    if (parser.seen('X'))
        tune_axis(AxisEnum::X_AXIS, cur, feedrate);
    if (parser.seen('Y'))
        tune_axis(AxisEnum::Y_AXIS, cur, feedrate);
    if (parser.seen('Z'))
        tune_axis(AxisEnum::Z_AXIS, cur, feedrate);
}

#endif