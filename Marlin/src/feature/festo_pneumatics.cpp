#include "../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../module/planner.h"

#    include "festo_pneumatics.h"

static constexpr float TANK_PRESSURE_TARGET = 100.0f;
static constexpr float TANK_PRESSURE_MAX = 200.0f;

namespace pneumatics {

//
// init
//

void init()
{
    OUT_WRITE(PRESSURE_VALVE_C1_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_C2_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_C3_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_LID_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_LID2_PIN, PRESSURE_VALVE_OPEN_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_PUMP_OUT_PIN, PRESSURE_VALVE_OPEN_LEVEL);
    OUT_WRITE(PRESSURE_PUMP_EN_PIN, LOW);

    SET_INPUT(PRESSURE_REGULATOR_SENSE_PIN);
    SET_INPUT(PRESSURE_TANK_PIN);
    SET_INPUT(GRIPPER_VACUUM_PIN);

    set_regulator(3.0f);
}

//
// Pressure Regulation
//

void set_regulator(float kPa)
{
    // 500kPa regulator 5V analog input, 12 bit DAC
    static constexpr float pressure_factor = 20.4f;
    uint32_t value = static_cast<uint32_t>(kPa * pressure_factor);
    analogWrite(PRESSURE_REGULATOR_PIN, value);
}

inline void pump_enable(bool enable)
{
    WRITE(PRESSURE_PUMP_EN_PIN, enable ? HIGH : LOW);
}

void pressurize_tank(millis_t timeout_after_ms)
{
    if (tank_pressure.read_avg() >= TANK_PRESSURE_MAX) {
        pump_enable(false);
        return;
    }
    pump_enable(true);
    millis_t timeout = millis() + timeout_after_ms;
    while (tank_pressure.read_avg() < TANK_PRESSURE_TARGET && millis() < timeout) {
        idle();
        delay(100);
    }
    pump_enable(false);
}

PressureToken::~PressureToken()
{
    --users;
    if (has_users())
        return;
    pressure_valves(false);
}

/**
 * @brief tell the pressure system you will be using pressure
 * 
 * @return a pressure token that will guarantee pressure is turned on until it goes out of scope
*/
[[nodiscard]] PressureToken use_pressure()
{
    if (!PressureToken::has_users())
        PressureToken::pressure_valves(true);
    return PressureToken{};
}

void update_tank()
{
    if (!PressureToken::has_users())
        pump_enable(false);
    pump_enable(tank_pressure.read_avg() < TANK_PRESSURE_TARGET);
}

//
// Mixing
//

constexpr static pin_t get_valve(uint8_t tool)
{
    switch (tool) {
    case 0:
        return PRESSURE_VALVE_C1_PIN;
    case 1:
        return PRESSURE_VALVE_C2_PIN;
    case 2:
        return PRESSURE_VALVE_C3_PIN;
    default:
        return NC;
    }
}

void apply_mixing_pressure(uint8_t tool)
{
    const pin_t pin = get_valve(tool);
    planner.synchronize();
    WRITE(pin, PRESSURE_VALVE_OPEN_LEVEL);
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
#    if ENABLED(CHECK_LID_GRIPPER_LOCATION_BEFORE_RELEASE)
    static constexpr xyz_pos_t safe_lid_drop_pos = LID_GRIPPER_RELEASE_LOCATION;
    if (current_position != safe_lid_drop_pos) {
        SERIAL_ERROR_MSG("printbed in wrong location for release!");
        return;
    }
#    endif
    auto _ = use_pressure();
    WRITE(PRESSURE_VALVE_LID_PIN, PRESSURE_VALVE_OPEN_LEVEL);
    static constexpr millis_t MIN_LID_RELEASE_DURATION = 1000; // ms
    millis_t gripper_rengage_time = millis() + MIN_LID_RELEASE_DURATION;
    while (PENDING(millis(), gripper_rengage_time))
        idle();
    WRITE(PRESSURE_VALVE_LID_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
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