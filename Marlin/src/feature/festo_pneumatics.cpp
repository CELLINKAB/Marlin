#include "../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../module/planner.h"

#    include "festo_pneumatics.h"

namespace pneumatics {

//
// constants
//
static constexpr float TANK_PRESSURE_TARGET = 75.0f;
static constexpr float TANK_PRESSURE_MAINTAINENCE = 50.0f;
static constexpr float TANK_PRESSURE_MAX = 100.0f;

static constexpr float TANK_GAIN = 0.183239119;
static constexpr float REGULATOR_GAIN = 0.04;
static constexpr float VACUUM_GAIN = -0.02;

static constexpr float TANK_OFFSET = 150.0f;
static constexpr float VACUUM_OFFSET = -16.5f;
static constexpr float REGULATOR_OFFSET = 0.0f;

//
// init
//

void init()
{
    OUT_WRITE(PRESSURE_VALVE_C1_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_C2_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_C3_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_LID_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_LID2_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_VALVE_PUMP_OUT_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    OUT_WRITE(PRESSURE_PUMP_EN_PIN, LOW);

    SET_INPUT(PRESSURE_REGULATOR_SENSE_PIN);
    SET_INPUT(PRESSURE_TANK_PIN);
    SET_INPUT(GRIPPER_VACUUM_PIN);

    set_regulator_pressure(5.0f);
}

//
// Pressure Regulation
//

static float regulator_set_pressure = 0;

void set_regulator_pressure(float kPa)
{
    // 500kPa regulator 5V analog input, 12 bit DAC
    static constexpr float pressure_factor = 1.0f / REGULATOR_GAIN;
    uint32_t value = static_cast<uint32_t>(kPa * pressure_factor);
    analogWrite(PRESSURE_REGULATOR_PIN, value);
    regulator_set_pressure = kPa;
}

float get_regulator_set_pressure()
{
    return regulator_set_pressure;
}

inline void pump_enable(bool enable)
{
    WRITE(PRESSURE_PUMP_EN_PIN, enable ? HIGH : LOW);
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

void pump_update()
{
    const auto current_pressure = tank_pressure.read_avg();
    const bool needs_pressure = (current_pressure < TANK_PRESSURE_MAINTAINENCE
                                 || (PressureToken::has_users()
                                     && current_pressure < TANK_PRESSURE_TARGET));
    PressureToken::pressure_valves(needs_pressure);
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
        return -1;
    }
}

void apply_mixing_pressure(uint8_t tool)
{
    const pin_t pin = get_valve(tool);
    planner.synchronize();
    WRITE(pin, PRESSURE_VALVE_OPEN_LEVEL);
    safe_delay(200);
}

void release_mixing_pressure(uint8_t tool)
{
    const pin_t pin = get_valve(tool);
    planner.synchronize();
    WRITE(pin, PRESSURE_VALVE_CLOSE_LEVEL);
    safe_delay(200);
}

//
// Lid Gripper
//
//  For grip
// V1 Off/close V2 On/Open V3 Off/close
// For release
// V1 ON/open V2 Off/close V3 ON/open
void set_gripper_valves(GripperState state)
{
    switch (state) {
    case GripperState::Release: {
        WRITE(PRESSURE_VALVE_LID_PIN, PRESSURE_VALVE_OPEN_LEVEL);
        WRITE(PRESSURE_VALVE_LID2_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
        break;
    }
    case GripperState::Grip:
        WRITE(PRESSURE_VALVE_LID_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
        WRITE(PRESSURE_VALVE_LID2_PIN, PRESSURE_VALVE_OPEN_LEVEL);
        break;
    case GripperState::Close:
        WRITE(PRESSURE_VALVE_LID_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
        WRITE(PRESSURE_VALVE_LID2_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
        break;
    }
}

void suck_lid()
{
    static constexpr float GRIP_VACUUM_THRESHOLD = -20.0f;
    if (PressureToken::has_users()) {
        SERIAL_ECHOLN("SOMETHING_USING_PRESSURE_DURING_LID_GRIP");
    }
    auto _ = pneumatics::use_pressure();
    WRITE(PRESSURE_VALVE_PUMP_OUT_PIN, PRESSURE_VALVE_CLOSE_LEVEL);
    const millis_t timeout = millis() + 5000;
    while (millis() < timeout && gripper_vacuum.read_avg() > GRIP_VACUUM_THRESHOLD) {
        idle();
    }
}

void update()
{
    static millis_t next_update = millis();
    if (millis() < next_update)
        return;

    gripper_vacuum.update();
    tank_pressure.update();
    regulator_feedback.update();

    pump_update();

    next_update += 100;
}

//
// Analog Pressure Sensor
//

AnalogPressureSensor::AnalogPressureSensor(pin_t sense_pin, float scale_factor, float offset_kPa)
    : scalar(scale_factor)
    , offset(offset_kPa)
    , pin(sense_pin)
    , avg_raw(0)
{
    pinMode(sense_pin, INPUT_ANALOG);
}

AnalogPressureSensor gripper_vacuum(GRIPPER_VACUUM_PIN, VACUUM_GAIN, VACUUM_OFFSET);
AnalogPressureSensor tank_pressure(PRESSURE_TANK_PIN, TANK_GAIN, TANK_OFFSET);
AnalogPressureSensor regulator_feedback(PRESSURE_REGULATOR_SENSE_PIN, REGULATOR_GAIN, REGULATOR_OFFSET);

} // namespace pneumatics

#endif // FESTO_PNEUMATICS