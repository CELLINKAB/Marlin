#include "../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "festo_pneumatics.h"

#    include "../module/planner.h"

namespace pneumatics {

//
// Pressure Regulation
//

void set_regulator(float kPa)
{
    static constexpr float pressure_factor = 20.4f; // temporary
    uint32_t value = static_cast<uint32_t>(kPa * pressure_factor);
    analogWrite(PRESSURE_REGULATOR_PIN, value);
}

//
// Mixing
//

constexpr static pin_t get_valve(uint8_t tool)
{
    switch (tool) {
        case 0: return PRESSURE_VALVE_C1_PIN;
        case 1: return PRESSURE_VALVE_C2_PIN;
        case 2: return PRESSURE_VALVE_C3_PIN;
        default: return NC;
    }
}

void apply_mixing_pressure(uint8_t tool)
{
    const pin_t pin = get_valve(tool);
    planner.synchronize();
    WRITE(pin, !PRESSURE_VALVE_CLOSE_LEVEL);
    delay(100);
}

void release_mixing_pressure(uint8_t tool)
{
    const pin_t pin = get_valve(tool);
    planner.synchronize();
    WRITE(pin, PRESSURE_VALVE_CLOSE_LEVEL);
    delay(100);
}

//
// Lid Gripper
//

void gripper_release()
{
    #if ENABLED(CHECK_LID_GRIPPER_LOCATION_BEFORE_RELEASE)
        static constexpr xyz_pos_t safe_lid_drop_pos = LID_GRIPPER_RELEASE_LOCATION;
        if (current_position != safe_lid_drop_pos) {
            SERIAL_ECHOLN("printbed in wrong location for release!");
            return;
        }
    #endif
    WRITE(GRIPPER_VACUUM_PIN, !PRESSURE_VALVE_CLOSE_LEVEL);
    delay(100);
    WRITE(GRIPPER_VACUUM_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
}

//
// Analog Pressure Sensor
//

AnalogPressureSensor::AnalogPressureSensor(pin_t sense_pin, float scale_factor, float offset_kPa)
    : scalar(scale_factor)
    , offset(offset_kPa)
    , pin(sense_pin)
{
    pinMode(sense_pin, INPUT_ANALOG);
}

float AnalogPressureSensor::read_avg(bool with_scaling, bool with_offset) const
{
    uint32_t samples[40]{};
    for (uint32_t& sample : samples) {
        sample = read_raw();
        delay(1);
    }
    float avg_raw = std::accumulate(std::cbegin(samples), std::cend(samples), 0.0f) / 40.0f;
    return apply_scaling_leveling(avg_raw, with_scaling, with_offset);
}

float AnalogPressureSensor::apply_scaling_leveling(float reading,
                                                   bool with_scaling,
                                                   bool with_offset) const
{
    return (reading * (static_cast<float>(with_scaling) * scalar))
           - (static_cast<float>(with_offset) * offset);
}

AnalogPressureSensor gripper_vacuum(GRIPPER_VACUUM_PIN);
AnalogPressureSensor tank_pressure(PRESSURE_TANK_PIN);
AnalogPressureSensor regulator_feedback(PRESSURE_REGULATOR_SENSE_PIN);

} // namespace pneumatics

#endif // FESTO_PNEUMATICS