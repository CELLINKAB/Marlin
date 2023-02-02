#include "../inc/MarlinConfig.h"

template<pin_t SENSOR, bool PULLUP = false>
struct DoorSensor
{
    DoorSensor()
    {
        if constexpr (PULLUP) {
            SET_INPUT_PULLUP(SENSOR);
        } else {
            SET_INPUT_PULLDOWN(SENSOR);
        }
        static auto report = []() {
            static bool debouncing = false;
            if (debouncing)
                return;
            debouncing = true;
            delay(10);
            static bool last_door_state = !READ(SENSOR);
            const bool door_state = READ(SENSOR);
            if (door_state != last_door_state) {
                SERIAL_ECHOLNPGM("DO:", DOOR_SENSOR_INVERTING != door_state);
            }
            last_door_state = door_state;
            debouncing = false;
        };
        attachInterrupt(SENSOR, static_cast<void (*)()>(report), CHANGE);
    }

    ~DoorSensor() { detachInterrupt(SENSOR); }
    DoorSensor(const DoorSensor&) = delete;
    DoorSensor(DoorSensor&&) = delete;
    DoorSensor& operator=(const DoorSensor&) = delete;
    DoorSensor& operator=(DoorSensor&&) = delete;
};