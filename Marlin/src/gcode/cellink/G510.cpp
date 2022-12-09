#include "../../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_AUTOCAL)

#    include "../../feature/optical_autocal.h"
#    include "../gcode.h"

#include "../../module/motion.h"

#    include <stdio.h>

static void update_offset(const xyz_pos_t& offset)
{
    position_shift -= offset;
    update_workspace_offset(AxisEnum::X_AXIS);
    update_workspace_offset(AxisEnum::Y_AXIS);
    update_workspace_offset(AxisEnum::Z_AXIS);
    if (WITHIN(GcodeSuite::active_coordinate_system, 0, MAX_COORDINATE_SYSTEMS - 1))
        GcodeSuite::coordinate_system[GcodeSuite::active_coordinate_system] = position_shift;
}

void GcodeSuite::G510()
{
    if (parser.seen('L')) {
        if (optical_autocal.is_calibrated(active_extruder)) {
            auto offset = optical_autocal.tool_change_offset(active_extruder);
            update_offset(offset);
        }
        return;
    }

    const auto feedrate = parser.feedrateval('F', AUTOCAL_DEFAULT_FEEDRATE);
    if (optical_autocal.full_autocal_routine(active_extruder, feedrate)) {
        update_offset(optical_autocal.offset(active_extruder));
    }
}

#endif