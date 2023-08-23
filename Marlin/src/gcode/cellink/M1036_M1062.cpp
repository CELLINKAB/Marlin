
#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../feature/air_system/pneumatics.h"
#    include "../../feature/cellink_reporter.h"
#    include "../gcode.h"

// set pressure regulator
void GcodeSuite::M1036()
{
    using namespace pneumatics;
    if (parser.seen('K'))
        regulator.set_point(parser.value_float());
    else {
        cellink::serial_echoln_kv("PREG_SET", regulator.set_point());
    }
}

void GcodeSuite::M1036_report(bool for_replay)
{
    using namespace pneumatics;
    report_heading_etc(for_replay, F("Air Pressure Regulator"));
    SERIAL_ECHOLNPGM("M1036 K", regulator.set_point());
}

// get pressure sensors
void GcodeSuite::M1062()
{
    pneumatics::report_sensors();
#    if ENABLED(AUTO_REPORT_PNEUMATIC_SENSORS)
    pneumatics::reporter.set_interval(parser.byteval('S'));
#    endif
}

template<typename SENSOR>
void set_sensor_params(SENSOR& sensor)
{
    if (parser.seen('O'))
        sensor.offset = parser.value_float();
    if (parser.seen('S'))
        sensor.scalar = parser.value_float();
}

// pressure sensor gain/offset
void GcodeSuite::M1100()
{
    if (parser.seen('L'))
        set_sensor_params(pneumatics::gripper_vacuum);
    else if (parser.seen('R'))
        set_sensor_params(pneumatics::regulator_feedback);
    else if (parser.seen('T'))
        set_sensor_params(pneumatics::tank_pressure);
    else {
        cellink::serial_echoln_kv("GRIP_S_OFFSET",
                                  pneumatics::gripper_vacuum.offset,
                                  "GRIP_S_GAIN",
                                  pneumatics::gripper_vacuum.scalar,
                                  "PREG_F_OFFSET",
                                  pneumatics::regulator_feedback.offset,
                                  "PREG_F_GAIN",
                                  pneumatics::regulator_feedback.scalar,
                                  "TANK_S_OFFSET",
                                  pneumatics::tank_pressure.offset,
                                  "TANK_S_GAIN",
                                  pneumatics::tank_pressure.scalar);

        return;
    }
}

#    define REPORT_M1100(letter, sensor) \
        SERIAL_ECHOLNPGM("M1100 " #letter " O", sensor.offset, " S", sensor.scalar);

void GcodeSuite::M1100_report(bool for_replay)
{
    report_heading_etc(for_replay, F("Air Pressure Sensors"));
    using namespace pneumatics;
    REPORT_M1100(L, gripper_vacuum);
    REPORT_M1100(R, regulator_feedback);
    REPORT_M1100(T, tank_pressure);
}

#endif // FESTO_PNEUMATICS