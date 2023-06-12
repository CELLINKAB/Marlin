

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

    constexpr float standard_deviation() const { return sqrt(variance); }
    constexpr uint16_t range() const { return (max > min) ? max - min : 0; }

    constexpr float z_test(float sample) const
    {
        return (sample - static_cast<float>(avg)) / standard_deviation();
    }

    void print() const
    {
        SERIAL_ECHOLNPGM("avg: ",
                         avg,
                         ", var: ",
                         variance,
                         ", std: ",
                         standard_deviation(),
                         "\nmin: ",
                         min,
                         ", max: ",
                         max,
                         ", range: ",
                         range());
    }
};

struct SweepResult
{
    float z_statistic;
    uint16_t sg_thresh;
};

uint16_t poll_sg_val(AxisEnum axis)
{
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
    TERN_(SENSORLESS_STALLGUARD_DELAY, safe_delay(SENSORLESS_STALLGUARD_DELAY));
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

    safe_delay(200);
    current_position[axis] += (distance * static_cast<float>(-home_dir(axis)));
    do_blocking_move_to(current_position);

    set_axis_current(axis, stored_current);
}

bool test_axis(AxisEnum axis, feedRate_t feedrate)
{
    constexpr static float TEST_BACKOFF_DISTANCE = 10.0f;

    planner.synchronize();

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
    static constexpr size_t MAX_SAMPLES = 32;
    std::array<uint16_t, MAX_SAMPLES> sg_samples;

    uint32_t sum = 0;

    millis_t next_poll = 0;
    for (auto& sample : sg_samples) {
        // spin until next result needed
        while (millis() <= next_poll)
            idle_no_sleep();
        next_poll = millis() + 10;
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

SweepResult test_sweep(AxisEnum axis, uint16_t cur, feedRate_t feedrate)
{
    do_backoff(axis, feedrate); // one second of travel time

    set_axis_current(axis, cur);

    SERIAL_ECHO("Axis: ");
    SERIAL_CHAR(AXIS_CHAR(axis));
    SERIAL_ECHOLNPGM(", feedrate: ", feedrate, ", current: ", cur);

    current_position[axis] += (feedrate * 2.0f * home_dir(axis));
    line_to_current_position(feedrate);
    const millis_t move_start_time = millis();
    while (millis() <= move_start_time + 300)
        idle();

    if (DEBUGGING(LEVELING))
        SERIAL_ECHOLN("-moving values-");
    auto move_summary = analyze_sweep(axis);
    while (millis() <= move_start_time + 1000)
        idle();
    if (DEBUGGING(LEVELING))
        SERIAL_ECHOLN("-stalling values-");
    auto stall_summary = analyze_sweep(axis);

    if (DEBUGGING(INFO)) {
        SERIAL_ECHOLN("--pre stall--");
        move_summary.print();
        SERIAL_ECHOLN("--post stall--");
        stall_summary.print();
    }

    SweepResult res{move_summary.z_test(stall_summary.avg),
                    static_cast<uint16_t>(stall_summary.avg / 2)};
    SERIAL_ECHOLNPGM("Z statistic of stall: ", res.z_statistic);

    return res;
}

void tune_axis(AxisEnum axis, uint16_t cur, feedRate_t feedrate, bool test_all)
{
    if (feedrate == 0)
        feedrate = homing_feedrate(axis);
    if (cur == 0) {
        constexpr static uint16_t DEFAULT_HOMING_CURRENT[] = {X_CURRENT_HOME,
                                                              Y_CURRENT_HOME,
                                                              Z_CURRENT_HOME};
        cur = DEFAULT_HOMING_CURRENT[axis];
    }
    auto move_cur = set_axis_current(axis, cur);

    auto best_sweep = test_sweep(axis, cur, feedrate);
    uint16_t optimal_current = cur;
    feedRate_t optimal_feedrate = feedrate;

    static constexpr float CRITICAL_VALUE = -3.5f; // 99.9% confidence / p < 0.1%
    uint16_t cur_increment = 50;
    while (cur < move_cur && cur > 100 && (best_sweep.z_statistic > CRITICAL_VALUE || test_all)) {
        cur += cur_increment;
        auto new_sweep = test_sweep(axis, cur, feedrate);
        if (new_sweep.z_statistic < best_sweep.z_statistic) {
            best_sweep = new_sweep;
            optimal_current = cur;
        } else if (new_sweep.z_statistic > (best_sweep.z_statistic + 0.6f) && cur_increment == 50) {
            // getting worse, try the other way
            cur_increment = -50;
        }
    }
    cur = optimal_current;

    static constexpr feedRate_t MAX_FEEDRATE = 36.0f;
    static constexpr feedRate_t MIN_FEEDRATE = 3.0f;
    feedRate_t feedrate_increment = 5.0f;
    while (feedrate < MAX_FEEDRATE && feedrate > MIN_FEEDRATE
           && (best_sweep.z_statistic > CRITICAL_VALUE || test_all)) {
        feedrate += feedrate_increment;
        auto new_sweep = test_sweep(axis, cur, feedrate);
        if (new_sweep.z_statistic < best_sweep.z_statistic) {
            best_sweep = new_sweep;
            optimal_feedrate = feedrate;
        } else if (new_sweep.z_statistic > (best_sweep.z_statistic + 0.6f)
                   && feedrate_increment == 5.0f) { // getting worse, change direction
            feedrate_increment = -2.5f;
        }
    }
    feedrate = optimal_feedrate;

    set_axis_sg_thresh(axis, best_sweep.sg_thresh);

    if (test_axis(axis, feedrate))
        SERIAL_ECHOLNPGM("\nOptimal values:\nHOMING_CURRENT ",
                         optimal_current,
                         "\nHOMING_FEEDRATE ",
                         optimal_feedrate,
                         "\nSTALLGUARD_THRESHOLD ",
                         best_sweep.sg_thresh);
    else
        SERIAL_ECHOLN("Failed to tune axis");

    set_axis_current(axis, move_cur);
}

/**
 * @brief Automatically tune for best sensorless homing parameters
 * 
 */
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
    bool test_all = parser.boolval('A');

    TERN_(IMPROVE_HOMING_RELIABILITY, auto motion_states = begin_slow_homing());

    if (!parser.seen_axis()) {
        tune_axis(AxisEnum::X_AXIS, cur, feedrate, test_all);
        tune_axis(AxisEnum::Y_AXIS, cur, feedrate, test_all);
        tune_axis(AxisEnum::Z_AXIS, cur, feedrate, test_all);
        return;
    }

    if (parser.seen('X'))
        tune_axis(AxisEnum::X_AXIS, cur, feedrate, test_all);
    if (parser.seen('Y'))
        tune_axis(AxisEnum::Y_AXIS, cur, feedrate, test_all);
    if (parser.seen('Z'))
        tune_axis(AxisEnum::Z_AXIS, cur, feedrate, test_all);

    TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(motion_states));
}

#endif