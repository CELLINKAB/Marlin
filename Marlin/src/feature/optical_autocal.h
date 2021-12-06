
#include "../MarlinCore.h"
#include "../module/planner.h"
#include "../module/motion.h"
#include "../gcode/gcode.h"

#include <array>
#include <numeric>


struct OpticalAutocal
{
    inline static constexpr xyz_pos_t START_POSITION{AUTOCAL_START_POSITION};
    inline static constexpr xyz_pos_t END_POSITION_PRINTBED_DELTA{AUTOCAL_PRINTBED_CENTER_DELTA};
    static constexpr uint8_t NUM_CYCLES = 2;
    inline static constexpr xy_pos_t XY_OFFSET_ERR{-1.0f, -1.0f};
    static constexpr float Z_OFFSET_ERR{-1.0f};
    static constexpr pin_t SENSOR_1{OPTICAL_SENSOR_1_PIN};
    static constexpr pin_t SENSOR_2{OPTICAL_SENSOR_2_PIN};
    using YSweepArray = std::array<float, NUM_CYCLES>;

    OpticalAutocal()
    {
        SET_INPUT(SENSOR_1);
        SET_INPUT(SENSOR_2);
    }

    bool full_autocal_routine(float feedrate)
    {
        home_if_needed();
        do_blocking_move_to_z(START_POSITION.z + POST_AUTOCAL_SAFE_Z_HEIGHT);
        do_blocking_move_to_xy(START_POSITION);
        do_blocking_move_to_z(START_POSITION.z);
        planner.synchronize();

        const bool success = full_sensor_sweep(feedrate);
        if (!success)
        {
            do_blocking_move_to(START_POSITION);
            SERIAL_ERROR_MSG("autocalibration failed!");
        }
        else if (DEBUGGING(LEVELING) || DEBUGGING(INFO))
            SERIAL_ECHOLNPAIR("Nozzle offset: ", tool_offset);

        return success;
    }

private:
    static constexpr float SHORT_Y_RANGE = 6.0f;
    static constexpr float FULL_Y_RANGE = 25.0f;
    static constexpr float COARSE_Z_INCREMENT = 4.0f;
    static constexpr float MEDIUM_Z_INCREMENT = 1.0f;
    static constexpr float FINE_Z_INCREMENT = 0.125f;
    static constexpr float PRECISE_Z_INCREMENT = 0.025f;

    xyz_pos_t tool_offset{0};

    /**
     * @brief Perform a multi-step sweep of optical sensors to find precise tool offset
     * 
     * @param z_increment 
     * @param feedrate mm/s
     * @param cycles 
     */
    const bool full_sensor_sweep(const float feedrate)
    {
        const float z_offset = find_z_offset(feedrate);
        if (z_offset == Z_OFFSET_ERR)
            return false;
        else if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
            SERIAL_ECHOLNPAIR("Z offset: ", z_offset);

        do_blocking_move_to_z(z_offset + MEDIUM_Z_INCREMENT); // ensure nozzle is visible to both sensors

        const xy_pos_t xy_offset = find_xy_offset(feedrate);
        if (xy_offset == XY_OFFSET_ERR)
            return false;
        if (DEBUGGING(INFO) || DEBUGGING(LEVELING))
            SERIAL_ECHOLNPAIR("XY offset: ", xy_offset);

        do_blocking_move_to_xy_z(xy_offset, z_offset + MEDIUM_Z_INCREMENT);

        const bool sensor_1_check = READ(SENSOR_1);
        const bool sensor_2_check = READ(SENSOR_2);

        if (sensor_1_check && sensor_2_check) {
            tool_offset.set(xy_offset.x + END_POSITION_PRINTBED_DELTA.x,
                            xy_offset.y + END_POSITION_PRINTBED_DELTA.y,
                            z_offset + END_POSITION_PRINTBED_DELTA.z
                            );
            if (DEBUGGING(INFO) || DEBUGGING(LEVELING)) 
                print_pos(tool_offset, "Calibrated tool offset:");
            do_blocking_move_to_z(tool_offset.z + POST_AUTOCAL_SAFE_Z_HEIGHT);
            do_blocking_move_to_xy(tool_offset);
            do_blocking_move_to(tool_offset);
            //planner.set_position_mm({0.0,0.0,0.0});
            
        }
        else
            SERIAL_ERROR_MSG("Autocalibration succeeded but sanity check failed!"
                             "\nsensor 1: ", sensor_1_check,
                             "\nsensor 2: ", sensor_2_check);

        return true;
    }

    /**
     * @brief sweep in y direction across both sensors to derive nozzle centerpoint distance between beams
     *        then use that value to calculate the precise offset for both X and Y using sensor geometry
     * 
     * @param feedrate mm/s
     * @return const xy_pos_t XY coordinate of the intersection of both optical sensors
     */
    const xy_pos_t find_xy_offset(const float feedrate) const
    {
        volatile float sensor_1_trigger_y_pos{0.0f};
        volatile float sensor_2_trigger_y_pos{0.0f};
        volatile bool read_sensor_1 = false;
        volatile bool read_sensor_2 = false;

        const unsigned int delay_5mm = static_cast<int>(5000.0f / feedrate ?: feedrate_mm_s);

        // enable sensors
        auto isr1 = [&sensor_1_trigger_y_pos, &read_sensor_1, &read_sensor_2, delay_5mm]
        {
            const float y = planner.get_axis_positions_mm().y;
            if (!read_sensor_1)
                return;
            sensor_1_trigger_y_pos = y;
            read_sensor_1 = false;
            delay(delay_5mm);
            read_sensor_2 = true;
            if DEBUGGING(LEVELING) SERIAL_ECHOLNPAIR("sensor 1 triggered Y", y);
        };
        auto isr2 = [&sensor_2_trigger_y_pos, &read_sensor_2, &read_sensor_1, delay_5mm]
        {
            const float y = planner.get_axis_positions_mm().y;
            if (!read_sensor_2)
                return;
            sensor_2_trigger_y_pos = y;
            read_sensor_2 = false;
            delay(delay_5mm);
            read_sensor_1 = true;
            if DEBUGGING(LEVELING) SERIAL_ECHOLNPAIR("sensor 2 triggered Y", y);
        };

        attachInterrupt(SENSOR_1, isr1, RISING);
        attachInterrupt(SENSOR_2, isr2, RISING);

        // y1 - cross sensor 1 forwards; y2 - cross sensor 2 forwards
        // y3 - cross sensor 2 backwards; y4 - cross sensor 1 backwards
        YSweepArray y1{0.0f};
        YSweepArray y2{0.0f};
        YSweepArray y3{0.0f};
        YSweepArray y4{0.0f};

        for (size_t i = 0; i < NUM_CYCLES; ++i)
        {
            read_sensor_1 = true;
            read_sensor_2 = false;

            do_blocking_move_to_y(START_POSITION.y + FULL_Y_RANGE, feedrate);
            y1[i] = sensor_1_trigger_y_pos;
            y2[i] = sensor_2_trigger_y_pos;

            read_sensor_2 = true;
            read_sensor_1 = false;

            do_blocking_move_to_y(START_POSITION.y, feedrate);
            y3[i] = sensor_2_trigger_y_pos;
            y4[i] = sensor_1_trigger_y_pos;

            if DEBUGGING (LEVELING)
                SERIAL_ECHOLNPAIR(
                    "sweep: ", i,
                    " y1: ", y1[i],
                    " y2: ", y2[i],
                    " y3: ", y3[i],
                    " y4: ", y4[i]);
        }

        detachInterrupt(SENSOR_1);
        detachInterrupt(SENSOR_2);

        auto check_non_zero = [](const auto containter) -> const bool
        {
            return std::any_of(containter.cbegin(), containter.cend(), [](const float v)
                               { return v == 0.0f; });
        };

        const bool any_non_zero = check_non_zero(y1) || check_non_zero(y2) || check_non_zero(y3) || check_non_zero(y4);

        if (any_non_zero)
        {
            SERIAL_ERROR_MSG("zero values in sweep! Check optical sensors.","Calibration aborted.");
            return XY_OFFSET_ERR;
        }

        auto cycles_avg = [](const auto s1, const auto s2) -> const float
        {
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

        const float x = START_POSITION.x - xy_offset;
        const float y = nozzle_y1 + xy_offset;

        return {x, y};
    }

    float scan_for_tip(float z, const float inc, bool &condition, const float feedrate) const
    {
        while (!condition && z < soft_endstop.max.z)
        {
            do_blocking_move_to_z(z, feedrate);
            do_blocking_move_to_y(START_POSITION.y + SHORT_Y_RANGE, feedrate);
            do_blocking_move_to_y(START_POSITION.y, feedrate);
            z += inc;
        }

        if (DEBUGGING(ERRORS) && z >= soft_endstop.max.z)
            SERIAL_ERROR_MSG("No nozzle found during Z sweep!");
        else if DEBUGGING (LEVELING)
            SERIAL_ECHOLNPAIR("Z sweep increment=", inc, "; found nozzle at z=", z);

        condition = false;
        return (z - inc) - inc; // report position before interrupt triggered
    }

    const float find_z_offset(const float feedrate) const
    {
        float z = START_POSITION.z;
        bool triggered = false;

        attachInterrupt(
            SENSOR_1, [&]
            { triggered = true; },
            RISING);

        z = scan_for_tip(z, COARSE_Z_INCREMENT, triggered, feedrate);
        z = scan_for_tip(z, MEDIUM_Z_INCREMENT, triggered, feedrate);
        z = scan_for_tip(z, FINE_Z_INCREMENT, triggered, feedrate);
        z = scan_for_tip(z, PRECISE_Z_INCREMENT, triggered, feedrate);
        z += PRECISE_Z_INCREMENT;

        detachInterrupt(SENSOR_1);

        if (z >= (soft_endstop.max.z - PRECISE_Z_INCREMENT))
            z = Z_OFFSET_ERR;
        
        return z;
    }
};

#if ENABLED(OPTICAL_AUTOCAL)
extern OpticalAutocal optical_autocal;
#endif