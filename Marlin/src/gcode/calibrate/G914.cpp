

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
        TERN(Y_DUAL_ENDSTOPS,
             return (stepperY.SG_RESULT() + stepperY2.SG_RESULT()) / 2,
             return stepperY.SG_RESULT());
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

constexpr void set_homing_current(AxisEnum axis, uint16_t cur)
{
    switch (axis) {
    case AxisEnum::X_AXIS:
        stepperX.homing_current = cur;
        TERN_(HAS_DUAL_X_STEPPERS, stepperX2.homing_current = cur);
        break;
    case AxisEnum::Y_AXIS:
        stepperY.homing_current = cur;
        TERN_(HAS_DUAL_Y_STEPPERS, stepperY2.homing_current = cur);
        break;
    case AxisEnum::Z_AXIS:
        stepperZ.homing_current = cur;
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

enum class SanityTestResult {
    Ok,
    FalsePositive,
    FalseNegative,
    NoTrigger,
};

SanityTestResult test_hit(AxisEnum axis, feedRate_t feedrate)
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
        return SanityTestResult::NoTrigger;

    endstops.hit_on_purpose();

    constexpr static float ERROR_MARGIN = 0.1f; // mm
    auto position_error = (planner.triggered_position_mm(axis) - start_pos) * -home_dir(axis);
    if (DEBUGGING(INFO))
        SERIAL_ECHOLNPGM("axis test position error: ", position_error);
    if (position_error > ERROR_MARGIN) {
        current_position[axis] += position_error * home_dir(axis);
        do_blocking_move_to(current_position, feedrate);
        return SanityTestResult::FalsePositive;
    } else if (position_error < (-ERROR_MARGIN * 10)) {
        return SanityTestResult::FalseNegative;
    } else
        return SanityTestResult::Ok;
}

SummaryStats analyze_sweep(AxisEnum axis)
{
    static constexpr size_t MAX_SAMPLES = 36;
    std::array<uint16_t, MAX_SAMPLES> sg_samples;

    uint32_t sum = 0;

    millis_t next_poll = 0;
    for (auto& sample : sg_samples) {
        // spin until next result needed
        while (millis() <= next_poll)
            idle_no_sleep();
        next_poll = millis() + 8;
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
    while (millis() <= move_start_time + 1200)
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

    auto ideal_sg = (stall_summary.avg > stall_summary.standard_deviation())
                        ? (stall_summary.avg - stall_summary.standard_deviation())
                        : stall_summary.avg;

    SweepResult res{move_summary.z_test(stall_summary.avg), static_cast<uint16_t>(ideal_sg / 2)};
    SERIAL_ECHOLNPGM("Z statistic of stall: ", res.z_statistic);

    return res;
}

void tune_axis(AxisEnum axis, uint16_t cur, feedRate_t feedrate, bool test_all, bool dry_run)
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

    static constexpr float CRITICAL_VALUE = -9.0f; // 1/10000000 expected error rate
    uint16_t cur_increment = 20;
    while (cur <= move_cur && cur > 100 && (best_sweep.z_statistic >= CRITICAL_VALUE || test_all)) {
        cur += cur_increment;
        auto new_sweep = test_sweep(axis, cur, feedrate);
        if (new_sweep.z_statistic < best_sweep.z_statistic) {
            best_sweep = new_sweep;
            optimal_current = cur;
        } else if ((new_sweep.z_statistic > (best_sweep.z_statistic + 2.5f) || cur >= move_cur) && cur_increment == 20
                   && (!test_all || cur >= move_cur)) {
            // getting worse, try the other way
            cur_increment = -5;
            while (cur > move_cur)
                cur += cur_increment;
        }
    }
    cur = optimal_current;
    best_sweep.z_statistic = max(best_sweep.z_statistic, CRITICAL_VALUE);

    static constexpr feedRate_t MAX_FEEDRATE = 36.0f;
    static constexpr feedRate_t MIN_FEEDRATE = 6.0f;
    feedRate_t feedrate_increment = 5.0f;
    while (feedrate < MAX_FEEDRATE && feedrate > MIN_FEEDRATE
           && (best_sweep.z_statistic >= CRITICAL_VALUE || test_all)) {
        feedrate += feedrate_increment;
        auto new_sweep = test_sweep(axis, cur, feedrate);
        if (new_sweep.z_statistic < best_sweep.z_statistic) {
            best_sweep = new_sweep;
            optimal_feedrate = feedrate;
        } else if (new_sweep.z_statistic > (best_sweep.z_statistic + 2.0f) && feedrate_increment == 5.0f
                   && (!test_all || feedrate >= MAX_FEEDRATE)) { // getting worse, change direction
            feedrate_increment = -2.0f;
            while (feedrate >= MAX_FEEDRATE)
                feedrate += feedrate_increment;
        }
    }
    feedrate = optimal_feedrate;

    if (dry_run)
        return;

    static constexpr int16_t SG_MIN = 5;
    static constexpr int16_t SG_MAX = 250;
    size_t retries = 0;
    size_t good_retries = 0;
    while (retries++ < 10 && WITHIN(best_sweep.sg_thresh, 0, 255)) {
        set_axis_sg_thresh(axis, best_sweep.sg_thresh);
        auto test_result = test_hit(axis, feedrate);
        switch (test_result) {
        case SanityTestResult::Ok:
            if (good_retries < 3) {
                --retries;
                ++good_retries;
                break;
            } else {
                SERIAL_ECHOLNPGM("\nOptimal values:\nHOMING_CURRENT ",
                                 optimal_current,
                                 "\nHOMING_FEEDRATE ",
                                 optimal_feedrate,
                                 "\nSTALLGUARD_THRESHOLD ",
                                 best_sweep.sg_thresh);

                set_axis_current(axis, move_cur);
                homing_feedrate_mm_m[axis] = MMS_TO_MMM(optimal_feedrate);
                set_homing_current(axis, optimal_current);
                planner.synchronize();
                static constexpr xyz_pos_t post_home_backoff = HOMING_BACKOFF_POST_MM;
                current_position[axis] += post_home_backoff[axis];
                do_blocking_move_to(current_position);
                return;
            }
        case SanityTestResult::FalsePositive:
            if (best_sweep.sg_thresh >= SG_MIN)
                best_sweep.sg_thresh -= 5;
            break;
        case SanityTestResult::FalseNegative:
            [[fallthrough]];
        case SanityTestResult::NoTrigger:
            if (best_sweep.sg_thresh <= SG_MAX)
            best_sweep.sg_thresh += 2;
            break;
        }
    }

    SERIAL_ECHOLN("No optimal stallguard settings found. :(");
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
    for (size_t seconds_until_start = 10; seconds_until_start > 0; --seconds_until_start) {
        SERIAL_ECHOLN(seconds_until_start);
        safe_delay(1000);
    }

    feedRate_t feedrate = parser.feedrateval('F');
    auto cur = parser.ushortval('C');
    bool test_all = parser.boolval('A');
    bool dry_run = parser.boolval('D');

    TERN_(IMPROVE_HOMING_RELIABILITY, auto motion_states = begin_slow_homing());

    if (!parser.seen_axis()) {
        tune_axis(AxisEnum::X_AXIS, cur, feedrate, test_all, dry_run);
        tune_axis(AxisEnum::Y_AXIS, cur, feedrate, test_all, dry_run);
        tune_axis(AxisEnum::Z_AXIS, cur, feedrate, test_all, dry_run);
        return;
    }

    LOOP_NUM_AXES(i)
    {
        if (parser.seen(AXIS_CHAR(i)))
            tune_axis(static_cast<AxisEnum>(i), cur, feedrate, test_all, dry_run);
    }

    TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(motion_states));
}

#endif