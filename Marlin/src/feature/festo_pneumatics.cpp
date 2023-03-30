#include "../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../module/planner.h"

#    include "festo_pneumatics.h"

namespace pneumatics {

//
// constants
//

static constexpr float TANK_GAIN = 0.183239119;
static constexpr float REGULATOR_GAIN = 0.04;
static constexpr float VACUUM_GAIN = -0.02;

static constexpr float TANK_OFFSET = 150.0f;
static constexpr float VACUUM_OFFSET = -16.5f;
static constexpr float REGULATOR_OFFSET = 0.0f;


//
// statics
//

Pump<PRESSURE_PUMP_EN_PIN, PRESSURE_VALVE_PUMP_OUT_PIN> pump;

AnalogPressureSensor gripper_vacuum(GRIPPER_VACUUM_PIN, VACUUM_GAIN, VACUUM_OFFSET);
AnalogPressureSensor tank_pressure(PRESSURE_TANK_PIN, TANK_GAIN, TANK_OFFSET);
AnalogPressureSensor regulator_feedback(PRESSURE_REGULATOR_SENSE_PIN, REGULATOR_GAIN, REGULATOR_OFFSET);

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

    SET_INPUT(PRESSURE_REGULATOR_SENSE_PIN);
    SET_INPUT(PRESSURE_TANK_PIN);
    SET_INPUT(GRIPPER_VACUUM_PIN);

    pump.init();

    // warm up pressure sensors
    for (int i = 0; i < 20; ++i) {
        tank_pressure.update();
        regulator_feedback.update();
        gripper_vacuum.update();
    }

    set_regulator_pressure(5.0f);
}

void update()
{
    gripper_vacuum.update();
    tank_pressure.update();
    regulator_feedback.update();

    static millis_t next_update = millis();
    if (millis() < next_update)
        return;
    pump.update();
    next_update = millis() + 250;
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
    WRITE(pin, PRESSURE_VALVE_OPEN_LEVEL);
}

void release_mixing_pressure(uint8_t tool)
{
    const pin_t pin = get_valve(tool);
    WRITE(pin, PRESSURE_VALVE_CLOSE_LEVEL);
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



} // namespace pneumatics

#endif // FESTO_PNEUMATICS