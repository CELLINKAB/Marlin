#include "../inc/MarlinConfig.h"

template<pin_t SENSOR, bool INVERTING, bool PULLUP = false>
struct DoorSensor
{
    DoorSensor()
    {
        if constexpr (PULLUP) {
            SET_INPUT_PULLUP(SENSOR);
        } else {
            SET_INPUT_PULLDOWN(SENSOR);
        };
        attachInterrupt(SENSOR, report, CHANGE);
    }

    static bool is_open() { return static_cast<bool>(READ(DOOR_PIN)) ^ INVERTING; }

    static void report()
    {
        static bool debouncing = false;
        if (debouncing)
            return;
        debouncing = true;
        safe_delay(10);
        static bool last_door_state = is_open();
        const bool door_state = is_open();
        if (door_state != last_door_state) {
            SERIAL_ECHOLNPGM("DO:", door_state);
        }
        last_door_state = door_state;
        debouncing = false;
    }

    ~DoorSensor() { detachInterrupt(SENSOR); }
    DoorSensor(const DoorSensor&) = delete;
    DoorSensor(DoorSensor&&) = delete;
    DoorSensor& operator=(const DoorSensor&) = delete;
    DoorSensor& operator=(DoorSensor&&) = delete;
};