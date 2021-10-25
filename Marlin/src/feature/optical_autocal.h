
#include "../MarlinCore.h"
#include "../module/planner.h"

static constexpr xyz_pos_t START_POSITION = AUTOCAL_START_POSITION;
static constexpr float UNTRIGGERED_FEEDRATE = ((xyz_pos_t)DEFAULT_MAX_FEEDRATE).y;

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

    void full_sensor_sweep(const float z_increment, const float feedrate, uint8_t cycles) const
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
        attachInterrupt(SENSOR, isr, RISING);

        float z = START_POSITION.z;
        cycles = (!cycles) | cycles;  // guarantees non-zero
        while (--cycles)
        {
            const float pass_feedrate = feedrate * static_cast<float>(triggered);
            single_sensor_pass(z, pass_feedrate);

            // if this is the first cycle that triggered, redo it at desired rate
            const bool first_trigger = (triggered && pass_feedrate == 0.0);
            cycles += first_trigger;
            z += z_increment * static_cast<float>(!first_trigger);
        }
        detachInterrupt(SENSOR);
    }

    inline void single_sensor_pass(const float z, const float feedrate) const
    {
        do_blocking_move_to_z(z, feedrate);
        do_blocking_move_to_xy(START_POSITION.x, START_POSITION.y + Y_RANGE, feedrate);
        do_blocking_move_to_xy(START_POSITION.x, START_POSITION.y, feedrate);
        // there will be more math here eventually
    }


};

#if ENABLED(OPTICAL_AUTOCAL)
  extern OpticalAutocal<OPTICAL_AUTOCAL_PIN> optical_autocal;
#endif