#include "../inc/MarlinConfig.h"

template<pin_t SENSOR>
struct DoorSensor
{
    DoorSensor()
    {
        SET_INPUT_PULLUP(SENSOR);
        static auto report = []() {
                delayMicroseconds(100); // allow signal to settle
                const bool door_state = READ(SENSOR);
                SERIAL_ECHOLNPGM("DO:", door_state);
            };
        attachInterrupt(
            SENSOR,
            static_cast<void (*)()>(report),
            CHANGE);
    }

    ~DoorSensor() { detachInterrupt(SENSOR); }
    DoorSensor(const DoorSensor &) = delete;
    DoorSensor(DoorSensor &&) = delete;
    DoorSensor & operator=(const DoorSensor &) = delete;
    DoorSensor & operator=(DoorSensor &&) = delete;
};