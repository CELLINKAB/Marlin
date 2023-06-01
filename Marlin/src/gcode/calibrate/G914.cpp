

#include "../gcode.h"

#if ENABLED(SENSORLESS_HOMING)

#    include "../../module/endstops.h"
#    include "../../module/stepper/trinamic.h"
#    include "../parser.h"

#    include <array>
#    include <numeric>

struct SummaryStats
{
    uint32_t avg;
    float variance;
    uint16_t min;
    uint16_t max;

    constexpr inline float standard_deviation() { return sqrt(variance); }
    constexpr inline uint16_t range() { return max - min; }
    constexpr inline uint16_t recommended_sg()
    {
        const int16_t half_minimum = static_cast<int16_t>(min / 2);
        if (half_minimum > standard_deviation())
            return half_minimum - static_cast<uint16_t>(standard_deviation());
        else
            return 0;
    }
};

uint16_t poll_sg_val(AxisEnum axis)
{
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
}

uint16_t set_axis_current(AxisEnum axis, uint16_t mA = 0)
{
    if (mA == 0) {
        static constexpr uint16_t DEFAULTS[] = {X_CURRENT, Y_CURRENT, Z_CURRENT};
        mA = DEFAULTS[axis];
    }
    uint16_t ret_cur = 0;
    switch (axis) {
    case AxisEnum::X_AXIS:
        ret_cur = stepperX.getMilliamps();
        stepperX.rms_current(mA);
        TERN_(HAS_DUAL_X_STEPPERS, stepperX2.rms_current(mA));
        break;
    case AxisEnum::Y_AXIS:
        ret_cur = stepperY.getMilliamps();
        stepperY.rms_current(mA);
        TERN_(HAS_DUAL_Y_STEPPERS, stepperY2.rms_current(mA));
        break;
    case AxisEnum::Z_AXIS:
        ret_cur = stepperZ.getMilliamps();
        stepperZ.rms_current(mA);
        // TODO: add multi Z support
        break;
    default:
        break;
    }
    return ret_cur;
}

void set_axis_sg_thresh(AxisEnum axis, uint16_t thresh)
{
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
}

void do_backoff(AxisEnum axis, float distance)
{
    planner.synchronize();

    auto stored_current = set_axis_current(axis);

    current_position[axis] += (distance * static_cast<float>(home_dir(axis)));
    do_blocking_move_to(current_position);

    set_axis_current(axis, stored_current);
}

bool test_axis(AxisEnum axis, feedRate_t feedrate)
{
    constexpr static float TEST_BACKOFF_DISTANCE = 10.0f;

    float start_pos = planner.get_axis_position_mm(axis);

    do_backoff(axis, TEST_BACKOFF_DISTANCE);

    auto states = start_sensorless_homing_per_axis(axis);
    endstops.enable();

    current_position[axis] += (TEST_BACKOFF_DISTANCE * 2 * static_cast<float>(home_dir(axis)));
    do_blocking_move_to(current_position, feedrate);

    endstops.enable(false);
    end_sensorless_homing_per_axis(axis, states);

    if (!endstops.trigger_state())
        return false;

    endstops.hit_on_purpose();
    SERIAL_ECHOLNPGM("axis test position error: ", planner.triggered_position_mm(axis) - start_pos);
    return true;
}

SummaryStats analyze_sweep(AxisEnum axis)
{
    static constexpr size_t MAX_SAMPLES = 64;
    std::array<uint16_t, MAX_SAMPLES> sg_samples;

    uint32_t sum = 0;

    for (auto& sample : sg_samples) {
        sample = poll_sg_val(axis);
        sum += sample;
        if (DEBUGGING(LEVELING))
            SERIAL_ECHOLNPGM("SG:", sample);
    }

    uint32_t avg = sum / MAX_SAMPLES;

    float square_difference_sum = 0.0f;
    for (const auto sample : sg_samples) {
        square_difference_sum += powf(static_cast<float>(sample) - static_cast<float>(avg), 2.0f);
    }

    float variance = square_difference_sum / static_cast<float>(MAX_SAMPLES);

    auto [min_p, max_p] = std::minmax_element(sg_samples.cbegin(), sg_samples.cend());

    return SummaryStats{avg, variance, *min_p, *max_p};
}

void tune_axis(AxisEnum axis, uint16_t cur, feedRate_t feedrate)
{
    if (feedrate == 0)
        feedrate = homing_feedrate(axis);
    if (cur == 0) {
        constexpr static uint16_t DEFAULT_HOMING_CURRENT[] = {X_CURRENT_HOME,
                                                              Y_CURRENT_HOME,
                                                              Z_CURRENT_HOME};
        cur = DEFAULT_HOMING_CURRENT[axis];
    }

    do_backoff(axis, feedrate); // one second of travel time

    auto move_cur = set_axis_current(axis, cur);

    SERIAL_ECHO("Axis: ");
    SERIAL_CHAR(AXIS_CHAR(axis));
    SERIAL_EOL();
    SERIAL_ECHOLNPGM("feedrate: ", feedrate, ", current: ", cur);

    current_position[axis] += (feedrate * home_dir(axis));
    planner.buffer_segment(current_position, feedrate);
    safe_delay(200); // make sure we're into the move
    auto summary = analyze_sweep(axis);

    if (DEBUGGING(INFO))
        SERIAL_ECHOLNPGM("avg: ",
                         summary.avg,
                         ", var: ",
                         summary.variance,
                         ", std: ",
                         summary.standard_deviation(),
                         "\nmin: ",
                         summary.min,
                         ", max: ",
                         summary.max,
                         ", range: ",
                         summary.range());

    SERIAL_ECHOLNPGM("SG_THRESHOLD: ", summary.recommended_sg());

    set_axis_sg_thresh(axis, summary.recommended_sg());
    bool tuning_success = test_axis(axis, feedrate);
    SERIAL_ECHOLNPGM("Tuning trigger test: ", tuning_success);

    set_axis_current(axis, move_cur);
}

void GcodeSuite::G914()
{
    planner.finish_and_disable();
    set_all_unhomed();

    SERIAL_ECHOLN("Manually move the printbed to the home position");
    for (size_t seconds_until_start = 5; seconds_until_start > 0; --seconds_until_start) {
        SERIAL_ECHOLN(seconds_until_start);
        safe_delay(1000);
    }

    feedRate_t feedrate = parser.feedrateval('F');
    auto cur = parser.ushortval('C');

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