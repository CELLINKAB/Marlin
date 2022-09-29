
#include "../MarlinCore.h"

#if ENABLED(OPTICAL_AUTOCAL)

#    include "optical_autocal.h"

OpticalAutocal::OpticalAutocal()
    : tool_offset()
{
    SET_INPUT(SENSOR_1);
    SET_INPUT(SENSOR_2);
}

bool OpticalAutocal::full_autocal_routine(float feedrate)
{
    home_if_needed();
    do_blocking_move_to(START_POSITION);
    planner.synchronize();

    const bool success = full_sensor_sweep(feedrate);
    if (!success) {
        do_blocking_move_to(START_POSITION);
        SERIAL_ERROR_MSG("autocalibration failed!");
    } else if (DEBUGGING(LEVELING) || DEBUGGING(INFO))
        SERIAL_ECHOLNPGM("Nozzle offset: ", tool_offset);

    return success;
}

bool OpticalAutocal::is_calibrated() const
{
    static constexpr xyz_pos_t default_pos{};
    return tool_offset != default_pos;
}

const xyz_pos_t& OpticalAutocal::offset() const
{
    return tool_offset;
}

void OpticalAutocal::reset()
{
  tool_offset = xyz_pos_t{};
}

bool OpticalAutocal::full_sensor_sweep(const float feedrate)
{
    const float z_offset = find_z_offset(feedrate);
    if (z_offset == Z_OFFSET_ERR)
        return false;
    else if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
        SERIAL_ECHOLNPGM("Z offset: ", z_offset);

    do_blocking_move_to_z(z_offset - MEDIUM_Z_INCREMENT); // ensure nozzle is visible to both sensors

    const xy_pos_t xy_offset = find_xy_offset(feedrate);
    if (xy_offset == XY_OFFSET_ERR)
        return false;

    do_blocking_move_to_xy_z(xy_offset, z_offset - MEDIUM_Z_INCREMENT);

    const bool sensor_1_check = READ(SENSOR_1);
    const bool sensor_2_check = READ(SENSOR_2);

    if (!(sensor_1_check && sensor_2_check)) {
        SERIAL_ERROR_MSG("Autocalibration succeeded but sanity check failed!"
                         "\nsensor 1: ",
                         sensor_1_check,
                         "\nsensor 2: ",
                         sensor_2_check);
        //return false;
    }

    tool_offset.set(xy_offset.x + END_POSITION_PRINTBED_DELTA.x,
                    xy_offset.y + END_POSITION_PRINTBED_DELTA.y,
                    z_offset + END_POSITION_PRINTBED_DELTA.z);
    if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
        print_pos(tool_offset, F("Calibrated tool offset:"));
    do_blocking_move_to_z(tool_offset.z + POST_AUTOCAL_SAFE_Z_HEIGHT);
    do_blocking_move_to_xy(tool_offset);
    // planner.set_position_mm({0.0,0.0,0.0});

    return true;
}

xy_pos_t OpticalAutocal::find_xy_offset(const float feedrate) const
{
    volatile float sensor_1_trigger_y_pos{0.0f};
    volatile float sensor_2_trigger_y_pos{0.0f};
    volatile bool read_sensor_1 = false;
    volatile bool read_sensor_2 = false;

    const unsigned int delay_5mm = static_cast<int>(5000.0f / feedrate ?: feedrate_mm_s);

    // enable sensors
    auto isr1 = [&sensor_1_trigger_y_pos, &read_sensor_1, &read_sensor_2, delay_5mm] {
        const float y = planner.get_axis_positions_mm().y;
        if (!read_sensor_1)
            return;
        sensor_1_trigger_y_pos = y;
        read_sensor_1 = false;
        delay(delay_5mm);
        read_sensor_2 = true;
        if DEBUGGING (LEVELING)
            SERIAL_ECHOLNPGM("sensor 1 triggered Y", y);
    };
    auto isr2 = [&sensor_2_trigger_y_pos, &read_sensor_2, &read_sensor_1, delay_5mm] {
        const float y = planner.get_axis_positions_mm().y;
        if (!read_sensor_2)
            return;
        sensor_2_trigger_y_pos = y;
        read_sensor_2 = false;
        delay(delay_5mm);
        read_sensor_1 = true;
        if DEBUGGING (LEVELING)
            SERIAL_ECHOLNPGM("sensor 2 triggered Y", y);
    };

    attachInterrupt(SENSOR_1, isr1, RISING);
    attachInterrupt(SENSOR_2, isr2, RISING);

    // y1 - cross sensor 1 forwards; y2 - cross sensor 2 forwards
    // y3 - cross sensor 2 backwards; y4 - cross sensor 1 backwards
    YSweepArray y1{};
    YSweepArray y2{};
    YSweepArray y3{};
    YSweepArray y4{};

    for (size_t i = 0; i < NUM_CYCLES; ++i) {
        read_sensor_1 = true;
        read_sensor_2 = false;

        do_blocking_move_to_y(START_POSITION.y + FULL_Y_RANGE, feedrate);
        y1[i] = sensor_1_trigger_y_pos;
        y2[i] = sensor_2_trigger_y_pos;

        read_sensor_1 = false;
        read_sensor_2 = true;

        do_blocking_move_to_y(START_POSITION.y, feedrate);
        y3[i] = sensor_2_trigger_y_pos;
        y4[i] = sensor_1_trigger_y_pos;

        if DEBUGGING (LEVELING)
            SERIAL_ECHOLNPGM("sweep: ", i, " y1: ", y1[i], " y2: ", y2[i], " y3: ", y3[i], " y4: ", y4[i]);
    }

    detachInterrupt(SENSOR_1);
    detachInterrupt(SENSOR_2);

    auto check_non_zero = [](const auto containter) -> bool {
        return std::any_of(containter.cbegin(), containter.cend(), [](const float v) {
            return v == 0.0f;
        });
    };

    const bool any_non_zero = check_non_zero(y1) || check_non_zero(y2) || check_non_zero(y3)
                              || check_non_zero(y4);

    if (any_non_zero) {
        SERIAL_ERROR_MSG("zero values in sweep! Check optical sensors.", "Calibration aborted.");
        return XY_OFFSET_ERR;
    }

    auto cycles_avg = [](const auto s1, const auto s2) -> float {
        const float sum_s1 = std::accumulate(s1.cbegin(), s1.cend(), 0.0f);
        const float sum_s2 = std::accumulate(s2.cbegin(), s2.cend(), 0.0f);
        const float avg = (sum_s1 + sum_s2) / (s1.size() + s2.size());
        return avg;
    };

    const float nozzle_y1 = cycles_avg(y1, y4);
    const float nozzle_y2 = cycles_avg(y2, y3);

    const float dy = ABS(nozzle_y1 - nozzle_y2);

    // sensors cross at a 90 degree angle, which creates two congruent isoscles
    // right triangles with legs in the X and Y directions, both of value dy/2
    const float xy_offset = dy / 2.0f;

    if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
        SERIAL_ECHOLNPGM("XY offset: ", xy_offset);

    const float x = START_POSITION.x - xy_offset;
    const float y = nozzle_y1 + xy_offset;

    return {x, y};
}

float OpticalAutocal::scan_for_tip(float z, const float inc, bool& condition, const float feedrate) const
{
    while (!condition && z > soft_endstop.min.z) {
        do_blocking_move_to_z(z, feedrate);
        do_blocking_move_to_y(START_POSITION.y + SHORT_Y_RANGE, feedrate);
        do_blocking_move_to_y(START_POSITION.y, feedrate);
        z -= inc;
    }

    if (DEBUGGING(ERRORS) && z <= soft_endstop.min.z)
        SERIAL_ERROR_MSG("No nozzle found during Z sweep!");
    else if DEBUGGING (LEVELING)
        SERIAL_ECHOLNPGM("Z sweep increment=", inc, "; found nozzle at z=", z);

    condition = false;
    return (z + inc) + inc; // report position before interrupt triggered
}

float OpticalAutocal::find_z_offset(const float feedrate) const
{
    float z = START_POSITION.z;
    bool triggered = false;

    attachInterrupt(
        SENSOR_1, [&] { triggered = true; }, RISING);

    z = scan_for_tip(z, COARSE_Z_INCREMENT, triggered, feedrate);
    z = scan_for_tip(z, MEDIUM_Z_INCREMENT, triggered, feedrate);
    z = scan_for_tip(z, FINE_Z_INCREMENT, triggered, feedrate);
    z = scan_for_tip(z, PRECISE_Z_INCREMENT, triggered, feedrate);
    z -= PRECISE_Z_INCREMENT;

    detachInterrupt(SENSOR_1);

    if (z <= (soft_endstop.min.z + PRECISE_Z_INCREMENT))
        z = Z_OFFSET_ERR;

    return z;
}

OpticalAutocal optical_autocal;

#endif