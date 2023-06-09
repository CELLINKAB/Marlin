
#include "../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_AUTOCAL)

#    include <cmath>

#    include "optical_autocal.h"

float OpticalAutocal::x_offset_factor = 1.0f;

uint32_t OpticalAutocal::sensor_polarity = RISING;

/**
 * @brief Find and set offsets for given tool or else give an error
 * 
 * @param start_pos Where routine should start searching for nozzle
 * @param feedrate mm/s
 * @return ErrorCode
 */
auto OpticalAutocal::full_autocal_routine(const xyz_pos_t start_pos, const feedRate_t feedrate)
    -> ErrorCode
{
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

/**
 * @brief Checks if given tool has already had calibration performed on it
 * 
 * @param tool tool/extruder/hotend index
 * @return true Given tool has been calibrated at least once since machine start
 * @return false Given tool has not been calibrated
 */
[[nodiscard]] bool OpticalAutocal::is_calibrated(const uint8_t tool) const
{
    static constexpr xyz_pos_t default_pos{};
    return offsets[tool] != default_pos;
}

/**
 * @brief Gets the offsets applied by calibration, or default value if not calibrated
 * 
 * @param tool tool/extruder/hotend index
 * @return const xyz_pos_t& calibration offsets for tool
 */
[[nodiscard]] const xyz_pos_t& OpticalAutocal::offset(const uint8_t tool) const
{
    return offsets[tool];
}

/**
 * @brief Reset the calibration state of given tool to default such that is_calibrated will return false
 * 
 * @param tool tool/extruder/hotend indexs
 */
void OpticalAutocal::reset(const uint8_t tool)
{
    offsets[tool] = xyz_pos_t{};
}

/**
 * @brief Clear all calibration data, resets calibration to power on state.
 * 
 */
void OpticalAutocal::reset_all()
{
    offsets.fill(xyz_pos_t{});
}

/**
 * @brief Machine meta calibration routine to account for possible poor sensor alignment
 * 
 * @param start_pos Where the routine should start looking for the nozzle
 * @param feedrate  mm/s
 */
void OpticalAutocal::calibrate(xyz_pos_t start_pos, feedRate_t feedrate)
{
    constexpr static float linearity_tolerance = 0.01f; // one whole step per 5mm

    // find Z
    do_blocking_move_to(start_pos);
    start_pos.z = find_z_offset(start_pos.z, feedrate) - MEDIUM_Z_INCREMENT;
    do_blocking_move_to(start_pos);

    LongSweepCoords mid = long_sweep(feedrate);
    do_blocking_move_to_x(start_pos.x - 1.0f);
    LongSweepCoords start = long_sweep(feedrate);
    do_blocking_move_to_x(start_pos.x + 1.0f);
    LongSweepCoords end = long_sweep(feedrate);

    if (mid.has_zeroes() || start.has_zeroes() || end.has_zeroes())
        SERIAL_ERROR_MSG("Zero values in sweeps! Cannot calculate sensor angle.");

    float calculated_y_delta_mid = (end.y_delta() + start.y_delta()) / 2;

    if (!WITHIN(mid.y_delta(),
                calculated_y_delta_mid - linearity_tolerance,
                calculated_y_delta_mid + linearity_tolerance)) {
        SERIAL_ERROR_MSG("Sensors not linear! Cannot calculate sensor angle.");
    }

    float sensor1_slope = (end.y1() - start.y1()) / 2.0f;
    float sensor2_slope = (end.y2() - start.y2()) / 2.0f;

    float sensor_angle = std::atan(ABS(sensor1_slope)) + std::atan(ABS(sensor2_slope));
    x_offset_factor = (1.0f / tan(sensor_angle / 2.0f));

    float x_intercept_delta = (mid.y_delta() / ABS(sensor1_slope - sensor2_slope));
    float sensor_x_intercept = start_pos.x - x_intercept_delta;
    float sensor_y_intercept = mid.y1() - (x_intercept_delta * sensor1_slope);

    float alt_sensor_y_intercept = mid.y2() - (x_intercept_delta * sensor2_slope);
    if (!WITHIN(sensor_y_intercept,
                alt_sensor_y_intercept - linearity_tolerance,
                alt_sensor_y_intercept + linearity_tolerance)) {
        SERIAL_ERROR_MSG("Sensor Y-intercepts do not match! Cannot calculate intersection point.");
    }
    nozzle_calibration_extra_offset.x = END_POSITION_PRINTBED_DELTA.x + sensor_x_intercept;
    nozzle_calibration_extra_offset.y = END_POSITION_PRINTBED_DELTA.y + sensor_y_intercept;
    SERIAL_ECHOLNPGM("sensor 1 slope: ",
                     sensor1_slope,
                     ", sensor 2 slope: ",
                     sensor2_slope,
                     "\nCalculated angle (rad): ",
                     sensor_angle,
                     "\nX scale factor: ",
                     x_offset_factor,
                     "\nIntersection point: (",
                     sensor_x_intercept,
                     ", ",
                     sensor_y_intercept,
                     ")\nX offset: ",
                     nozzle_calibration_extra_offset.x,
                     ", Y offset: ",
                     nozzle_calibration_extra_offset.y);

    do_blocking_move_to_z(POST_AUTOCAL_SAFE_Z_HEIGHT);
}

/**
 * @brief Sweep in the Y direction which should pass through both sensors in both directions
 * 
 * @param feedrate_mm_s 
 * @return LongSweepCoords coordinates where the sensor triggered during sweep
 */
[[nodiscard]] auto OpticalAutocal::long_sweep(feedRate_t feedrate_mm_s) const -> LongSweepCoords
{
    volatile float sensor_1_trigger_y_pos{0.0f};
    volatile float sensor_2_trigger_y_pos{0.0f};
    LongSweepCoords retval{0.0f, 0.0f, 0.0f, 0.0f};
    const bool read_polarity = (sensor_polarity == RISING) ? HIGH : LOW;

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
                delay(1); // spin until clear of active sensor
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
                delay(1); // spin until clear of active sensor
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

/**
 * @brief Perform sweeps to obtain X Y and Z offsets
 * 
 * @param tool tool/extruder/hotend index to calibrate
 * @param start_pos where the routine should start to look for nozzle
 * @param feedrate mm/s
 * @return ErrorCode whether or not routine was successful and why
 */
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

/**
 * @brief Report sensor states to serial
 * 
 */
void OpticalAutocal::report_sensors() const
{
    const bool sensor_1_check = READ(SENSOR_1);
    const bool sensor_2_check = READ(SENSOR_2);
    SERIAL_ECHOLNPGM("sensor 1: ", sensor_1_check, "\nsensor 2: ", sensor_2_check);
}

/**
 * @brief Perform a series of sweeps to determine x and y offsets
 * 
 * @param start_pos where the routine should start to look for nozzle
 * @param feedrate mm/s
 * @return xy_pos_t offsets in mm towards the intersection of both beams (not final tool offset)
 */
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

    const float y_offset = delta_y / 2.0f;
    const float x_offset = y_offset * x_offset_factor;

    const float x = start_pos.x - (y_offset * x_offset_factor);
    const float y = avg_y1 + y_offset;

    if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
        SERIAL_ECHOLNPGM("X offset: ",
                         x_offset,
                         ", Y offset: ",
                         y_offset,
                         "\nCalculated intersection: (",
                         x,
                         ", ",
                         y,
                         ")");

    return {x, y};
}

/**
 * @brief Perform a series of sweeps in desired intervals to check at which Z height the nozzle is detected
 * 
 * @param z height to begin sweeps
 * @param inc how much should Z change between sweeps
 * @param condition A marker which should be set high by interrupt when nozzle is detected
 * @param feedrate mm/s
 * @return float last Z height before sweeo where nozzle was detected (smallest distance from nozzle for given increment)
 */
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

/**
 * @brief Perform a scan in decreasing increments/increasing precision to find exact Z height of nozzle tip
 * 
 * @param z height to begin searching for nozzle
 * @param feedrate mm/s
 * @return float Most precise Z of nozzle tip, to be used as Z offset
 */
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