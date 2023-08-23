#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../module/planner.h"
#    include "../cellink_reporter.h"

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

    // warm up pressure sensors
    for (int i = 0; i < 20; ++i) {
        tank_pressure.update();
        regulator_feedback.update();
        gripper_vacuum.update();
    }

    regulator.set_point(5.0f);
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
// Reporting
//

void report_sensors()
{
    cellink::serial_echoln_kv("REG",
                              regulator_feedback.read_avg(),
                              "TANK",
                              tank_pressure.read_avg(),
                              "GRIP",
                              gripper_vacuum.read_avg());
}

#    if ENABLED(AUTO_REPORT_PNEUMATIC_SENSORS)
void Reporter::report()
{
    report_sensors();
}
Reporter reporter;
#    endif

} // namespace pneumatics

#endif // FESTO_PNEUMATICS