
#include "../MarlinCore.h"
#include "../module/planner.h"

static constexpr xyz_pos_t START_POSITION = AUTOCAL_START_POSITION;
static constexpr uint8_t MAX_AUTOCAL_CYCLES = 200;
template <const pin_t SENSOR_1, const pin_t SENSOR_2>
struct OpticalAutocal
{

    OpticalAutocal()
    {
        SET_INPUT_PULLDOWN(SENSOR_1);
        SET_INPUT_PULLDOWN(SENSOR_2);
    }

    void full_autocal_routine(uint8_t cycles, float z_increment, float feedrate)
    {
        home_if_needed();
        do_blocking_move_to(START_POSITION);
        planner.synchronize();

        const bool success = full_sensor_sweep(z_increment, feedrate, cycles);
        if (!success) SERIAL_ERROR_MSG("autocalibration sanity check failed!\n Current postition: ", 
                                       planner.get_axis_positions_mm(), 
                                       "sensor 1: ", READ(SENSOR_1), 
                                       "sensor 2: ", READ(SENSOR_2)
                                       );
    }

 private:
    static constexpr float SHORT_Y_RANGE = 6.0;
    static constexpr float FULL_Y_RANGE = 30.0;

    xyz_pos_t tool_offset{0};

    /**
     * @brief Perform a multi-step sweep of optical sensors to find precise tool offset
     * 
     * @param z_increment 
     * @param feedrate mm/s
     * @param cycles 
     */
    const bool full_sensor_sweep(const float z_increment, const float feedrate, uint8_t cycles)
    {
        const float z_offset = find_z_offset();

        do_blocking_move_to_z(z_offset + 0.025);  // ensure nozzle is visible to both sensors

        const xy_pos_t xy_offset = find_xy_offset(feedrate);

        do_blocking_move_to_xy_z(xy_offset, z_offset);

        const bool parameter_check = (READ(SENSOR_1) && READ(SENSOR_2));

        if (parameter_check) tool_offset.set(xy_offset, z_offset);

        return parameter_check;
    }

    /**
     * @brief sweep in y direction across both sensors to derive nozzle centerpoint distance between beams
     *        then use that value to calculate the precise offset for both X and Y using sensor geometry
     * 
     * @param feedrate mm/s
     * @return const float y distance between sensors
     */
    const xy_pos_t find_xy_offset(const float feedrate) const
    {
        float sensor_1_trigger_y_pos{0};
        float sensor_2_trigger_y_pos{0};
        // enable sensors
        auto isr1 = [&]{
            sensor_1_trigger_y_pos = planner.get_axis_positions_mm().copy().y;
        };
        attachInterrupt(SENSOR_1, isr1, RISING);
        auto isr2 = [&]{
            sensor_2_trigger_y_pos = planner.get_axis_positions_mm().copy().y;
        };
        attachInterrupt(SENSOR_2, isr2, RISING);

        do_blocking_move_to_y(START_POSITION.y + FULL_Y_RANGE, feedrate);
        const float y1 = sensor_1_trigger_y_pos;
        const float y2 = sensor_2_trigger_y_pos;
        do_blocking_move_to_y(START_POSITION.y, feedrate);            
        const float y3 = sensor_2_trigger_y_pos;
        const float y4 = sensor_1_trigger_y_pos;
        detachInterrupt(SENSOR_1);
        detachInterrupt(SENSOR_2);

        const float nozzle_y1 = (y1 + y4) / 2;
        const float nozzle_y2 = (y2 + y3) / 2;

        const float dy = ABS(nozzle_y1 - nozzle_y2);

        // sensors cross at a 90 degree angle, which creates two congruent isoscles 
        // right triangles with legs in the X and Y directions, both of value dy/2
        const float xy_offset = dy / 2;

        const float x = START_POSITION.x - xy_offset;
        const float y = nozzle_y1 + xy_offset;

        return {x, y};
    }

    const float scan_for_tip(float z, const float inc, bool & condition) const
    {
        while (!condition)
        {
            do_blocking_move_to_z(z);
            do_blocking_move_to_y(START_POSITION.y + SHORT_Y_RANGE);
            do_blocking_move_to_y(START_POSITION.y);
            z += inc;
        }
        z = z - inc - inc;  // go to the last position before interrupt
        do_blocking_move_to_z(z);
        condition = false;
        return z;
    }

    const float find_z_offset() const
    {
        float z = START_POSITION.z;
        bool triggered = false;

        attachInterrupt(SENSOR_1, [&]{
            triggered = true;
        }, RISING);

        float z_precision = 4.0;
        do
        {
            z = scan_for_tip(z, z_precision, triggered);
            z_precision /= 4;
        } while (z_precision > 0.025);

        detachInterrupt(SENSOR_1);
        return z;
    }


};

#if ENABLED(OPTICAL_AUTOCAL)
  extern OpticalAutocal<OPTICAL_SENSOR_1_PIN, OPTICAL_SENSOR_2_PIN> optical_autocal;
#endif