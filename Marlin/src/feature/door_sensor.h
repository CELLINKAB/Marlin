#include "../inc/MarlinConfig.h"

template<pin_t SENSOR>
struct DoorSensor
{
    DoorSensor()
    {
        SET_INPUT_PULLUP(SENSOR);
        attachInterrupt(
            SENSOR,
            []() {
                delayMicroseconds(100); // allow signal to settle
                const bool door_state = READ(SENSOR);
                SERIAL_ECHOLNPGM("DO:", door_state);
            },
            CHANGE);
    }

    ~DoorSensor() { detachInterrupt(SENSOR); }
    DoorSensor(const DoorSensor &) = delete;
    DoorSensor(DoorSensor &&) = delete;
    DoorSensor & operator=(const DoorSensor &) = delete;
    DoorSensor & operator=(DoorSensor &&) = delete;
};