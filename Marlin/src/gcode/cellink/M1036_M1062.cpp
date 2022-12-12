
#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../feature/festo_pneumatics.h"
#    include "../gcode.h"

#    include "cellink_reporting.h"

// set pressure regulator
void GcodeSuite::M1036()
{
    using namespace pneumatics;
    if (parser.seen('K'))
        set_regulator_pressure(parser.value_float());
    else {
        SERIAL_ECHOLN_CELLINK_KV("PREG_SET", get_regulator_set_pressure());
    }
}

// get pressure sensors
void GcodeSuite::M1062()
{
    using namespace pneumatics;
    SERIAL_ECHO_CELLINK_KV("REG", regulator_feedback.read_avg());
    SERIAL_ECHO_CELLINK_KV("TANK", tank_pressure.read_avg());
    SERIAL_ECHOLN_CELLINK_KV("GRIP", gripper_vacuum.read_avg());
}

// pressure sensor gain/offset
void GcodeSuite::M1100() {

    pneumatics::AnalogPressureSensor *sensor;
        if (parser.seen('L')) sensor = &pneumatics::gripper_vacuum;
        else if (parser.seen('R')) sensor = &pneumatics::regulator_feedback;
        else if (parser.seen('T')) sensor = &pneumatics::tank_pressure;
        else {
            SERIAL_ECHO_CELLINK_KV("GRIP_S_OFFSET", pneumatics::gripper_vacuum.offset);
            SERIAL_ECHO_CELLINK_KV("GRIP_S_GAIN", pneumatics::gripper_vacuum.scalar);
            SERIAL_ECHO_CELLINK_KV("PREG_F_OFFSET", pneumatics::regulator_feedback.offset);
            SERIAL_ECHO_CELLINK_KV("PREG_F_GAIN", pneumatics::regulator_feedback.scalar);
            SERIAL_ECHO_CELLINK_KV("TANK_S_OFFSET", pneumatics::tank_pressure.offset);
            SERIAL_ECHOLN_CELLINK_KV("TANK_S_GAIN", pneumatics::tank_pressure.scalar);

            return;
        }

    if (parser.seen('O'))
        sensor->offset = parser.value_float();
    if (parser.seen('S'))
        sensor->scalar = parser.value_float();

}

#endif // FESTO_PNEUMATICS