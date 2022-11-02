
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
        set_regulator(parser.value_float());
}

// get pressure sensors
void GcodeSuite::M1062()
{
    using namespace pneumatics;
    SERIAL_ECHO_CELLINK_KV("REG", regulator_feedback.read_avg());
    SERIAL_ECHO_CELLINK_KV("TANK", tank_pressure.read_avg());
    SERIAL_ECHOLN_CELLINK_KV("GRIP", gripper_vacuum.read_avg());
}

// pressure regulator offset
void GcodeSuite::M1100()
{
    
}

#endif // FESTO_PNEUMATICS