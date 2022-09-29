// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

#    include "../../module/motion.h"
#    include "../../module/planner.h"
#    include "../gcode.h"


/**
   * @brief Get current position in cellink protocol format
   * 
   */
void GcodeSuite::M1015()
{
    SERIAL_ECHOLNPGM("XPOS:", current_position.x, ",YPOS:", current_position.y, ",ZPOS:", current_position.z);
}

/**
   * @brief Get current machine position in cellink protocol format
   * 
   */
void GcodeSuite::M1016()
{
    const auto pos = planner.get_axis_positions_mm();
    SERIAL_ECHOLNPGM("XMPOS:", pos.x, ",YMPOS:", pos.y, ",ZMPOS:", pos.z);
}

#endif // CELLINK_REPORTING