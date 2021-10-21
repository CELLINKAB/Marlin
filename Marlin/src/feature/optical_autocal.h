
#include "../MarlinCore.h"
#include "../module/planner.h"

static constexpr xyz_pos_t START_POSITION = AUTOCAL_START_POSITION;

template <const pin_t SENSOR>
struct OpticalAutocal
{
    static constexpr uint8_t MAX_AUTOCAL_CYCLES = 200;

    OpticalAutocal()
    {
        SET_INPUT(SENSOR);
    }

    void full_autocal_routine(uint8_t cycles, float z_increment, float feedrate) const
    {
        home_if_needed();
        planner.synchronize();
        do_blocking_move_to(START_POSITION);
        full_sensor_sweep(z_increment, feedrate, cycles);
        do_blocking_move_to(START_POSITION);
        planner.synchronize();
    }

 private:
    static constexpr float Y_RANGE = 6.0;

    void full_sensor_sweep(const float z_increment, const float feedrate, const uint8_t cycles) const
    {
        bool triggered = false;
        auto isr = [&]{
            const auto position_at_interrupt = planner.get_axis_positions_mm().copy();
            const bool sensor_state = READ(SENSOR);
            triggered = true;
            SERIAL_ECHOLNPAIR("State:", sensor_state, 
                            " X:", position_at_interrupt.x, 
                            " Y:", position_at_interrupt.y, 
                            " Z:", position_at_interrupt.z
                            );
        };
        attachInterrupt(SENSOR, isr, CHANGE);
        for (uint8_t i = 0; i <= cycles; ++i)
        {
            const float z = START_POSITION.z + (i * z_increment);
            single_sensor_pass(z, feedrate, triggered);
        }
        detachInterrupt(SENSOR);
    }

    inline void single_sensor_pass(const float z, const float feedrate, bool& triggered) const
    {
        static constexpr float untriggered_y_feedrate = ((xyz_pos_t)DEFAULT_MAX_FEEDRATE).y;
        do_blocking_move_to_xy(START_POSITION.x, START_POSITION.y + Y_RANGE, feedrate);
        do_blocking_move_to_xy(START_POSITION.x, START_POSITION.y, triggered ? feedrate : untriggered_y_feedrate);
        do_blocking_move_to_z(z, feedrate);
        triggered = false;
        // there will be more math here eventually
    }


};

#if ENABLED(OPTICAL_AUTOCAL)
  extern OpticalAutocal<OPTICAL_AUTOCAL_PIN> optical_autocal;
#endif