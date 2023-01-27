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
            delayMicroseconds(100); // allow signal to settle
            const bool door_state = READ(SENSOR);
            SERIAL_ECHOLNPGM("DO:", door_state);
        };
        attachInterrupt(SENSOR, static_cast<void (*)()>(report), CHANGE);
    }

    ~DoorSensor() {detachInterrupt(SENSOR);}
    DoorSensor(const DoorSensor&) = delete;
    DoorSensor(DoorSensor&&) = delete;
    DoorSensor& operator=(const DoorSensor&) = delete;
    DoorSensor& operator=(DoorSensor&&) = delete;
};