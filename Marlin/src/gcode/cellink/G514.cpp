
#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../feature/festo_pneumatics.h"
#    include "../gcode.h"
#    include "../parser.h"
# include "../../module/planner.h"
#include "../../feature/guppi_printhead/chantarelle.h"

/**
 * @brief mixing extrude / pneumatic move
 * 
 */
void GcodeSuite::G514()
{
    uint8_t tool = get_target_extruder_from_command();
    get_destination_from_command();
    planner.synchronize();
    auto _ = pneumatics::use_pressure();
    pneumatics::apply_mixing_pressure(tool);    
    planner.buffer_line(destination, feedrate_mm_s, tool);
    planner.synchronize();
    pneumatics::release_mixing_pressure(tool);
}

void GcodeSuite::G515()
{
    pneumatics::gripper_release();
}

#endif // FESTO_PNEUMATICS
