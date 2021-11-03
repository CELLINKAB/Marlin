
#include "../MarlinCore.h"
#include "../module/planner.h"

#include <array>
#include <numeric>

static constexpr xyz_pos_t START_POSITION = AUTOCAL_START_POSITION;
static constexpr uint8_t MAX_AUTOCAL_CYCLES = 200;
static constexpr uint8_t NUM_CYCLES = 2;
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
        if (!success) {
            SERIAL_ERROR_MSG("autocalibration sanity check failed!\n Current postition: ", 
                                       planner.get_axis_positions_mm(), 
                                       "sensor 1: ", READ(SENSOR_1), 
                                       "sensor 2: ", READ(SENSOR_2)
                                       );
        } else {
            if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("Nozzle offset: ", tool_offset);
        }
    }

 private:
    static constexpr float SHORT_Y_RANGE = 6.0f;
    static constexpr float FULL_Y_RANGE = 30.0f;
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
    const bool full_sensor_sweep(const float z_increment, const float feedrate, uint8_t cycles)
    {
        const float z_offset = find_z_offset();
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("Z offset: ", z_offset);
        do_blocking_move_to_z(z_offset + FINE_Z_INCREMENT);  // ensure nozzle is visible to both sensors

        const xy_pos_t xy_offset = find_xy_offset(feedrate);
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPAIR("XY offset: ", xy_offset);

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
     * @return const xy_pos_t XY coordinate of the intersection of both optical sensors
     */
    const xy_pos_t find_xy_offset(const float feedrate) const
    {
        float sensor_1_trigger_y_pos{0};
        float sensor_2_trigger_y_pos{0};

        // y1 - cross sensor 1 forwards; y2 - cross sensor 2 forwards
        // y3 - cross sensor 2 backwards; y4 - cross sensor 1 backwards
        std::array<float, NUM_CYCLES> y1{0};
        std::array<float, NUM_CYCLES> y2{0};
        std::array<float, NUM_CYCLES> y3{0};
        std::array<float, NUM_CYCLES> y4{0};


        // enable sensors
        auto isr1 = [&]{
            sensor_1_trigger_y_pos = planner.get_axis_positions_mm().copy().y;
        };
        attachInterrupt(SENSOR_1, isr1, RISING);
        auto isr2 = [&]{
            sensor_2_trigger_y_pos = planner.get_axis_positions_mm().copy().y;
        };
        attachInterrupt(SENSOR_2, isr2, RISING);

        for (uint8_t i = 0; i < NUM_CYCLES; ++i)
        {
            do_blocking_move_to_y(START_POSITION.y + FULL_Y_RANGE, feedrate);
            y1[i] = sensor_1_trigger_y_pos;
            y2[i] = sensor_2_trigger_y_pos;
            do_blocking_move_to_y(START_POSITION.y, feedrate);            
            y3[i] = sensor_2_trigger_y_pos;
            y4[i] = sensor_1_trigger_y_pos;
            detachInterrupt(SENSOR_1);
            detachInterrupt(SENSOR_2);
        }
        const float nozzle_y1 = cycles_avg(y1, y4);
        const float nozzle_y2 = cycles_avg(y2, y3);

        const float dy = ABS(nozzle_y1 - nozzle_y2);

        // sensors cross at a 90 degree angle, which creates two congruent isoscles 
        // right triangles with legs in the X and Y directions, both of value dy/2
        const float xy_offset = dy / 2;

        const float x = START_POSITION.x - xy_offset;
        const float y = nozzle_y1 + xy_offset;

        return {x, y};
    }


    const float cycles_avg(const std::array<float, NUM_CYCLES> s1, const std::array<float, NUM_CYCLES> s2) const
    {
        const float sum_s1 = std::accumulate(s1.begin(), s1.end(), 0.0f);
        const float sum_s2 = std::accumulate(s2.begin(), s2.end(), 0.0f);
        const float avg = (sum_s1 + sum_s2) / (s1.size() + s2.size());
        return avg;
    }

    float scan_for_tip(float z, const float inc, bool & condition) const
    {
        while (!condition)
        {
            do_blocking_move_to_z(z);
            do_blocking_move_to_y(START_POSITION.y + SHORT_Y_RANGE);
            do_blocking_move_to_y(START_POSITION.y);
            z += inc;
        }
        condition = false;
        return (z - inc) - inc; // report position before interrupt triggered
    }

    const float find_z_offset() const
    {
        float z = START_POSITION.z;
        bool triggered = false;

        attachInterrupt(SENSOR_1, [&]{
            triggered = true;
        }, RISING);

        z = scan_for_tip(z, COARSE_Z_INCREMENT, triggered);
        z = scan_for_tip(z, MEDIUM_Z_INCREMENT, triggered);
        z = scan_for_tip(z, FINE_Z_INCREMENT, triggered);
        z = scan_for_tip(z, PRECISE_Z_INCREMENT, triggered);

        detachInterrupt(SENSOR_1);
        return z;
    }


};

#if ENABLED(OPTICAL_AUTOCAL)
  extern OpticalAutocal<OPTICAL_SENSOR_1_PIN, OPTICAL_SENSOR_2_PIN> optical_autocal;
#endif