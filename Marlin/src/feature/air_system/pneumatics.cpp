#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../module/planner.h"

#    include "pneumatics.h"

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
    // 500kPa regulator 5V analog input clipped to 3.3v, 12 bit DAC
    static constexpr float pressure_factor = 4096.0 / ((3.3 / 5.0) * 500.0);
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

} // namespace pneumatics

#endif // FESTO_PNEUMATICS