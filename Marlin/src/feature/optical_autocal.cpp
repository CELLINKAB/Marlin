
#include "../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_AUTOCAL)

#    include "optical_autocal.h"

uint32_t OpticalAutocal::sensor_polarity = RISING;

auto OpticalAutocal::full_autocal_routine(const xyz_pos_t start_pos, const feedRate_t feedrate)
    -> ErrorCode
{
    home_if_needed();
    do_blocking_move_to(start_pos);
    planner.synchronize();

    static auto set_polarity = [] {
        SET_INPUT_PULLUP(SENSOR_1);
        SET_INPUT_PULLUP(SENSOR_2);

        const auto sensor_1_polarity = READ(SENSOR_1);
        const auto sensor_2_polarity = READ(SENSOR_2);

        if (sensor_1_polarity != sensor_2_polarity) {
            return false;
        }

        OpticalAutocal::sensor_polarity = (sensor_1_polarity == LOW) ? RISING : FALLING;
        return true;
    };

    if (!set_polarity())
        return ErrorCode::POLARITY_MISMATCH;

    const ErrorCode result = full_sensor_sweep(active_extruder, start_pos, feedrate);

    do_blocking_move_to_z(POST_AUTOCAL_SAFE_Z_HEIGHT);

    return result;
}

[[nodiscard]] bool OpticalAutocal::is_calibrated(const uint8_t tool) const
{
    static constexpr xyz_pos_t default_pos{};
    return offsets[tool] != default_pos;
}

[[nodiscard]] const xyz_pos_t& OpticalAutocal::offset(const uint8_t tool) const
{
    return offsets[tool];
}

void OpticalAutocal::reset(const uint8_t tool)
{
    offsets[tool] = xyz_pos_t{};
}

void OpticalAutocal::reset_all()
{
    offsets.fill(xyz_pos_t{});
}

void OpticalAutocal::test(uint8_t cycles, xyz_pos_t start_pos, feedRate_t feedrate)
{
    constexpr static uint8_t MAX_CYCLES = 48;
    cycles = min(cycles, MAX_CYCLES);
    std::array<LongSweepCoords, MAX_CYCLES> coords{};

    SERIAL_ECHOLNPGM("running ", cycles, " sweeps...");

    LongSweepCoords avg_sweep{};
    LongSweepCoords min_sweep{1'000'000'000.0f, 1'000'000'000.0f, 1'000'000'000.0f, 1'000'000'000.0f};
    LongSweepCoords max_sweep{-1'000'000'000.0f,
                              -1'000'000'000.0f,
                              -1'000'000'000.0f,
                              -1'000'000'000.0f};

    do_blocking_move_to(start_pos);
    start_pos.z = find_z_offset(start_pos.z, feedrate) - MEDIUM_Z_INCREMENT;
    do_blocking_move_to(start_pos);

    uint8_t error_count = 0;

    for (uint8_t cycle_count = 0; cycle_count < cycles; ++cycle_count) {
        coords[cycle_count] = long_sweep(feedrate);
        if (coords[cycle_count].has_zeroes()) {
            ++error_count;
            continue;
        }
        if (DEBUGGING(INFO)) {
            SERIAL_ECHOPGM("sweep: ", cycle_count, ", ");
            coords[cycle_count].print();
        }

        avg_sweep.sensor_1_forward_y += coords[cycle_count].sensor_1_forward_y;
        avg_sweep.sensor_2_forward_y += coords[cycle_count].sensor_2_forward_y;
        avg_sweep.sensor_2_backward_y += coords[cycle_count].sensor_2_backward_y;
        avg_sweep.sensor_1_backward_y += coords[cycle_count].sensor_1_backward_y;

        if (coords[cycle_count].sensor_1_forward_y < min_sweep.sensor_1_forward_y)
            min_sweep.sensor_1_forward_y = coords[cycle_count].sensor_1_forward_y;
        if (coords[cycle_count].sensor_2_forward_y < min_sweep.sensor_2_forward_y)
            min_sweep.sensor_2_forward_y = coords[cycle_count].sensor_2_forward_y;
        if (coords[cycle_count].sensor_2_backward_y < min_sweep.sensor_2_backward_y)
            min_sweep.sensor_2_backward_y = coords[cycle_count].sensor_2_backward_y;
        if (coords[cycle_count].sensor_1_backward_y < min_sweep.sensor_1_backward_y)
            min_sweep.sensor_1_backward_y = coords[cycle_count].sensor_1_backward_y;

        if (coords[cycle_count].sensor_1_forward_y > max_sweep.sensor_1_forward_y)
            max_sweep.sensor_1_forward_y = coords[cycle_count].sensor_1_forward_y;
        if (coords[cycle_count].sensor_2_forward_y > max_sweep.sensor_2_forward_y)
            max_sweep.sensor_2_forward_y = coords[cycle_count].sensor_2_forward_y;
        if (coords[cycle_count].sensor_2_backward_y > max_sweep.sensor_2_backward_y)
            max_sweep.sensor_2_backward_y = coords[cycle_count].sensor_2_backward_y;
        if (coords[cycle_count].sensor_1_backward_y > max_sweep.sensor_1_backward_y)
            max_sweep.sensor_1_backward_y = coords[cycle_count].sensor_1_backward_y;
    }

    if (error_count >= cycles) {
        SERIAL_ECHOLN("No sweeps successful!");
        return;
    }

    avg_sweep.sensor_1_forward_y /= (cycles - error_count);
    avg_sweep.sensor_2_forward_y /= (cycles - error_count);
    avg_sweep.sensor_2_backward_y /= (cycles - error_count);
    avg_sweep.sensor_1_backward_y /= (cycles - error_count);

    // calculate variance
    LongSweepCoords sweep_variance;
    for (uint8_t cycle_count = 0; cycle_count < cycles; ++cycle_count) {
        if (coords[cycle_count].has_zeroes())
            continue;
        sweep_variance.sensor_1_forward_y += pow(coords[cycle_count].sensor_1_forward_y
                                                     - avg_sweep.sensor_1_forward_y,
                                                 2.0f);
        sweep_variance.sensor_2_forward_y += pow(coords[cycle_count].sensor_2_forward_y
                                                     - avg_sweep.sensor_2_forward_y,
                                                 2.0f);
        sweep_variance.sensor_2_backward_y += pow(coords[cycle_count].sensor_2_backward_y
                                                      - avg_sweep.sensor_2_backward_y,
                                                  2.0f);
        sweep_variance.sensor_1_backward_y += pow(coords[cycle_count].sensor_1_backward_y
                                                      - avg_sweep.sensor_1_backward_y,
                                                  2.0f);
    }

    sweep_variance.sensor_1_forward_y /= (cycles - error_count);
    sweep_variance.sensor_2_forward_y /= (cycles - error_count);
    sweep_variance.sensor_2_backward_y /= (cycles - error_count);
    sweep_variance.sensor_1_backward_y /= (cycles - error_count);

    LongSweepCoords sweep_deviation{sqrt(sweep_variance.sensor_1_forward_y),
                                    sqrt(sweep_variance.sensor_2_forward_y),
                                    sqrt(sweep_variance.sensor_2_backward_y),
                                    sqrt(sweep_variance.sensor_1_backward_y)};

    auto print_stats = [start_pos](LongSweepCoords coord) {
        constexpr static xyz_pos_t delta = AUTOCAL_PRINTBED_CENTER_DELTA;
        coord.print();
        SERIAL_ECHOLNPGM("y delta: ",
                         coord.y_delta(),
                         ", x offset: ",
                         ((start_pos.x + (coord.y_delta() / 2.0f)) + delta.x),
                         ", y offset: ",
                         ((coord.y1() + (coord.y_delta() / 2)) + delta.y));
    };

    SERIAL_ECHOLNPGM("Completed ",
                     cycles,
                     " with ",
                     error_count,
                     " sweeps containing errors and discarded");
    SERIAL_ECHOLN("--minimum--");
    print_stats(min_sweep);
    SERIAL_ECHOLN("--maximum--");
    print_stats(max_sweep);
    SERIAL_ECHOLN("--mean--");
    print_stats(avg_sweep);
    SERIAL_ECHOLN("--variance--");
    print_stats(sweep_variance);
    SERIAL_ECHOLN("--standard deviation--");
    print_stats(sweep_deviation);

    do_blocking_move_to_xy_z(start_pos, POST_AUTOCAL_SAFE_Z_HEIGHT);
}

xyz_pos_t OpticalAutocal::tool_change_offset(const uint8_t tool)
{
    xyz_pos_t new_offset{};

    if (is_calibrated(tool)) {
        new_offset = offsets[tool] - active_offset;
        active_offset = offsets[tool];
    }

    return new_offset;
}

[[nodiscard]] auto OpticalAutocal::long_sweep(feedRate_t feedrate_mm_s) const -> LongSweepCoords
{
    volatile float sensor_1_trigger_y_pos{0.0f};
    volatile float sensor_2_trigger_y_pos{0.0f};
    LongSweepCoords retval{0.0f, 0.0f, 0.0f, 0.0f};
    const bool read_polarity = (sensor_polarity == RISING) ? HIGH : LOW;

    // enable sensors
    auto isr1 = [&sensor_1_trigger_y_pos] {
        if (sensor_1_trigger_y_pos == 0.0f)
            sensor_1_trigger_y_pos = planner.get_axis_positions_mm().y;
    };
    auto isr2 = [&sensor_2_trigger_y_pos] {
        if (sensor_2_trigger_y_pos == 0.0f)
            sensor_2_trigger_y_pos = planner.get_axis_positions_mm().y;
    };

    attachInterrupt(SENSOR_1, isr1, sensor_polarity);
    current_position.y += FULL_Y_RANGE;
    planner.buffer_line(current_position, feedrate_mm_s);
    while (planner.busy()) {
        idle();
        // sensor 1 triggered, switch sensors
        if (sensor_1_trigger_y_pos != 0.0f) {
            while (READ(SENSOR_1) == read_polarity)
                delay(1);
            hal.isr_off();
            retval.sensor_1_forward_y = sensor_1_trigger_y_pos;
            detachInterrupt(SENSOR_1);
            attachInterrupt(SENSOR_2, isr2, sensor_polarity);
            sensor_1_trigger_y_pos = 0.0f;
            hal.isr_on();
        }
    }
    retval.sensor_2_forward_y = sensor_2_trigger_y_pos;
    sensor_2_trigger_y_pos = 0.0f;

    // switch direction
    current_position.y -= FULL_Y_RANGE;
    planner.buffer_line(current_position, feedrate_mm_s);
    while (planner.busy()) {
        idle();
        // sensor 2 triggered, switch sensors
        if (sensor_2_trigger_y_pos != 0.0f) {
            while (READ(SENSOR_2) == read_polarity)
                delay(1);
            hal.isr_off();
            retval.sensor_2_backward_y = sensor_2_trigger_y_pos;
            detachInterrupt(SENSOR_2);
            attachInterrupt(SENSOR_1, isr1, sensor_polarity);
            sensor_2_trigger_y_pos = 0.0f;
            hal.isr_on();
        }
    }
    detachInterrupt(SENSOR_1);
    retval.sensor_1_backward_y = sensor_1_trigger_y_pos;

    return retval;
}

[[nodiscard]] auto OpticalAutocal::full_sensor_sweep(const uint8_t tool,
                                                     const xyz_pos_t start_pos,
                                                     const float feedrate) -> ErrorCode
{
    const float z_offset = find_z_offset(start_pos.z, feedrate);
    if (z_offset == Z_OFFSET_ERR)
        return ErrorCode::NO_NOZZLE_DETECTED;
    else if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
        SERIAL_ECHOLNPGM("Z offset: ", z_offset);

    do_blocking_move_to_z(z_offset - MEDIUM_Z_INCREMENT); // ensure nozzle is visible to both sensors

    const xy_pos_t xy_offset = find_xy_offset(start_pos, feedrate);
    if (xy_offset == XY_OFFSET_ERR)
        return ErrorCode::CALIBRATION_FAILED;

    do_blocking_move_to_xy_z(xy_offset, z_offset - MEDIUM_Z_INCREMENT);

    const bool sensor_1_check = READ(SENSOR_1);
    const bool sensor_2_check = READ(SENSOR_2);

    ErrorCode retval = ErrorCode::OK;

    if (!(sensor_1_check && sensor_2_check)) {
        retval = ErrorCode::SANITY_CHECK_FAILED;
        if (DEBUGGING(ERRORS))
            report_sensors();
    }

    offsets[tool].set(xy_offset.x + END_POSITION_PRINTBED_DELTA.x,
                      xy_offset.y + END_POSITION_PRINTBED_DELTA.y,
                      z_offset + END_POSITION_PRINTBED_DELTA.z);

    if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
        print_pos(offsets[tool], F("Calibrated tool offset:"));

    do_blocking_move_to_z(POST_AUTOCAL_SAFE_Z_HEIGHT);

    return retval;
}

void OpticalAutocal::report_sensors() const
{
    const bool sensor_1_check = READ(SENSOR_1);
    const bool sensor_2_check = READ(SENSOR_2);
    SERIAL_ECHOLNPGM("sensor 1: ", sensor_1_check, "\nsensor 2: ", sensor_2_check);
}

[[nodiscard]] xy_pos_t OpticalAutocal::find_xy_offset(const xy_pos_t start_pos,
                                                      const float feedrate) const
{
    float delta_y = 0.0f;
    float avg_y1 = 0.0f;

    for (size_t i = 0; i < NUM_CYCLES; ++i) {
        LongSweepCoords sweep = long_sweep(feedrate);
        if (DEBUGGING(INFO)) {
            SERIAL_ECHOPGM("sweep: ", i, ", ");
            sweep.print();
        }
        if (sweep.has_zeroes()) {
            SERIAL_ERROR_MSG("zero values in sweep! Check optical sensors.", "Calibration aborted.");
            return XY_OFFSET_ERR;
        }
        delta_y += sweep.y_delta() / NUM_CYCLES; // iterative average
        avg_y1 += sweep.y1() / NUM_CYCLES;
    }

    // sensors cross at a 90 degree angle, which creates two congruent isosceles
    // right triangles with legs in the X and Y directions, both of value dy/2
    const float xy_offset = delta_y / 2.0f;

    if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
        SERIAL_ECHOLNPGM("XY offset: ", xy_offset);

    const float x = start_pos.x - xy_offset;
    const float y = avg_y1 + xy_offset;

    return {x, y};
}

[[nodiscard]] float OpticalAutocal::scan_for_tip(float z,
                                                 const float inc,
                                                 bool& condition,
                                                 const float feedrate) const
{
    while (!condition && z > soft_endstop.min.z) {
        do_blocking_move_to_z(z, feedrate);
        do_blocking_move_to_y(current_position.y + SHORT_Y_RANGE, feedrate);
        do_blocking_move_to_y(current_position.y - SHORT_Y_RANGE, feedrate);
        z -= inc;
    }

    if (DEBUGGING(ERRORS) && z <= soft_endstop.min.z)
        SERIAL_ERROR_MSG("No nozzle found during Z sweep!");
    else if DEBUGGING (LEVELING)
        SERIAL_ECHOLNPGM("Z sweep increment=", inc, "; found nozzle at z=", z);

    condition = false;
    return (z + inc) + inc; // report position before interrupt triggered
}

[[nodiscard]] float OpticalAutocal::find_z_offset(float z, const float feedrate) const
{
    bool triggered = false;

    attachInterrupt(
        SENSOR_1, [&] { triggered = true; }, sensor_polarity);

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