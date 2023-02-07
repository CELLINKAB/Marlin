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
    if (parser.seen('S')) {
        optical_autocal.report_sensors();
        return;
    }

    static constexpr xyz_pos_t DEFAULT_START_POS = AUTOCAL_START_POSITION;
    xyz_pos_t start_pos;
    start_pos.x = parser.axisunitsval('X', AxisEnum::X_AXIS, DEFAULT_START_POS.x);
    start_pos.y = parser.axisunitsval('Y', AxisEnum::Y_AXIS, DEFAULT_START_POS.y);
    start_pos.z = parser.axisunitsval('Z', AxisEnum::Z_AXIS, DEFAULT_START_POS.z);

    const auto feedrate = parser.feedrateval('F', AUTOCAL_DEFAULT_FEEDRATE);

    const xyz_pos_t existing_offset = optical_autocal.offset(active_extruder);
    switch (optical_autocal.full_autocal_routine(active_extruder, start_pos, feedrate)) {
        case OpticalAutocal::ErrorCode::OK:
            update_offset(optical_autocal.offset(active_extruder) - existing_offset);
            break;
        case OpticalAutocal::ErrorCode::CALIBRATION_FAILED:
            SERIAL_ECHO("NOZZLE_AUTOCALIBRATION_FAIL");
            break;
        case OpticalAutocal::ErrorCode::POLARITY_MISMATCH:
            SERIAL_ECHO("AUTOCAL_SENSOR_POLARITY_MISMATCH");
            break;
    }
}

#endif