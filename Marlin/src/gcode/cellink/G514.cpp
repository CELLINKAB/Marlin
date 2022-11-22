
#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../feature/festo_pneumatics.h"
#    include "../gcode.h"
#    include "../parser.h"
#include "../../feature/guppi_printhead/chantarelle.h"

/**
 * @brief mixing extrude / pneumatic move
 * 
 */
void GcodeSuite::G514()
{
    static constexpr float UL_STEPS_FACTOR = (1.0 / (4.6 * 4.6 * 3.14159265)) * ((float[])DEFAULT_AXIS_STEPS_PER_UNIT)[3];
    const auto tool = get_target_extruder_from_command();
    const int32_t steps = static_cast<int32_t>(parser.longval('E') * UL_STEPS_FACTOR);
    if (parser.seen('F')) ph_controller.set_extrusion_speed(static_cast<printhead::Index>(tool), parser.value_feedrate());
    auto _ = pneumatics::use_pressure();
    pneumatics::apply_mixing_pressure(tool);
    //G0_G1();
    ph_controller.add_raw_extruder_steps(static_cast<printhead::Index>(tool), steps);
    pneumatics::release_mixing_pressure(tool);
}

void GcodeSuite::G515()
{
    pneumatics::gripper_release();
}

#endif // FESTO_PNEUMATICS
