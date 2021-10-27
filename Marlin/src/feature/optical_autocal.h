
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
        SET_INPUT_PULLDOWN(SENSOR);
    }

    void full_autocal_routine(uint8_t cycles, float z_increment, float feedrate) const
    {
        home_if_needed();
        do_blocking_move_to(START_POSITION);
        planner.synchronize();
        full_sensor_sweep(z_increment, feedrate, cycles);
        do_blocking_move_to(START_POSITION);
    }

 private:
    static constexpr float Y_RANGE = 6.0;

    void full_sensor_sweep(const float z_increment, const float feedrate, uint8_t cycles) const
    {
        bool triggered = false;
        bool report = false;
        auto isr = [&]{
            const auto position_at_interrupt = planner.get_axis_positions_mm().copy();
            triggered = true;
            if (report) SERIAL_ECHOLNPAIR(" X:", position_at_interrupt.x, 
                              " Y:", position_at_interrupt.y, 
                              " Z:", position_at_interrupt.z
                              );
        };
        attachInterrupt(SENSOR, isr, RISING);

        float z = START_POSITION.z;
        cycles = (!cycles) | cycles;  // guarantees non-zero

        // go in large chunks to find rough needle bottom
        for (float inc = 4; inc >= z_increment; inc /= 8)
            scan_for_tip(z, inc, triggered);

        // enable the isr to report and begin measurement sweeps
        report = true;
        while (--cycles)
        {
            single_sensor_pass(z, feedrate);
            z += z_increment;
        }
        detachInterrupt(SENSOR);
    }

    inline void scan_for_tip(float & z, const float inc, bool & condition) const
    {
        while (!condition)
        {
            single_sensor_pass(z, 0);
            z += inc;
        }
        z = z - inc - inc;
        do_blocking_move_to_z(z);
        condition = false;
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