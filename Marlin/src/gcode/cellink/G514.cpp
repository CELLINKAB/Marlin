
#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../feature/festo_pneumatics.h"
#    include "../gcode.h"
#    include "../parser.h"

/**
 * @brief mixing extrude / pneumatic move
 * 
 */
void GcodeSuite::G514()
{
    const auto tool = get_target_extruder_from_command();
    pneumatics::pressurize_tank();
    pneumatics::apply_mixing_pressure(tool);
    G0_G1();
    pneumatics::release_mixing_pressure(tool);
}

void GcodeSuite::G515()
{
    pneumatics::gripper_release();
}

#endif // FESTO_PNEUMATICS
