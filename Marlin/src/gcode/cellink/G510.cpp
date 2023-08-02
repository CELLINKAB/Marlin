#include "../../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_AUTOCAL)

#    include "../../feature/bedlevel/bedlevel.h"
#    include "../../feature/optical_autocal.h"
#    include "../../module/motion.h"
#    include "../gcode.h"

#    include <cmath>
#    include <stdio.h>

xyz_pos_t OpticalAutocal::nozzle_calibration_extra_offset{};

void update_offset(const xyz_pos_t& offset)
{
    position_shift.x = -(offset.x - OpticalAutocal::nozzle_calibration_extra_offset.x);
    update_workspace_offset(AxisEnum::X_AXIS);
    position_shift.y = -(offset.y - OpticalAutocal::nozzle_calibration_extra_offset.y);
    update_workspace_offset(AxisEnum::Y_AXIS);
    position_shift.z = -(offset.z - OpticalAutocal::nozzle_calibration_extra_offset.z);
    update_workspace_offset(AxisEnum::Z_AXIS);
}

void reset_offset()
{
    position_shift.reset();
    update_workspace_offset(AxisEnum::X_AXIS);
    update_workspace_offset(AxisEnum::Y_AXIS);
    update_workspace_offset(AxisEnum::Z_AXIS);
}

void GcodeSuite::G510()
{
    // load offsets for new tool, used during tool change
    if (parser.seen('L')) {
        select_coordinate_system(active_extruder);

        if (optical_autocal.is_calibrated(active_extruder))
            update_offset(optical_autocal.offset(active_extruder));
        return;
    }

    // report sensor state for debugging
    if (parser.seen('S')) {
        optical_autocal.report_sensors();
        return;
    }

    // use coordinate system to match tool
    select_coordinate_system(active_extruder);

    // clear calibration on current printhead
    optical_autocal.reset(active_extruder);
    reset_offset();

    if (parser.seen('R') || homing_needed_error()) // 'R' resets offsets and leaves
        return;

    const bool leveling_active = planner.leveling_active;
    set_bed_leveling_enabled(false);
    Defer restore_leveling([leveling_active]() { set_bed_leveling_enabled(leveling_active); });

    static constexpr xyz_pos_t DEFAULT_START_POS = AUTOCAL_START_POSITION;
    xyz_pos_t start_pos;
    start_pos.x = parser.axisunitsval('X', AxisEnum::X_AXIS, DEFAULT_START_POS.x);
    start_pos.y = parser.axisunitsval('Y', AxisEnum::Y_AXIS, DEFAULT_START_POS.y);
    start_pos.z = parser.axisunitsval('Z', AxisEnum::Z_AXIS, DEFAULT_START_POS.z);

    const auto feedrate = parser.feedrateval('F', 25.0f);

    if (parser.seen('C')) {
        optical_autocal.calibrate(start_pos, feedrate);
        return;
    }

    switch (optical_autocal.full_autocal_routine(start_pos, feedrate)) {
    case OpticalAutocal::ErrorCode::SANITY_CHECK_FAILED:
        SERIAL_ECHOLN("AUTOCAL_SANITY_CHECK_FAIL");
        [[fallthrough]];
    case OpticalAutocal::ErrorCode::OK: {
        update_offset(optical_autocal.offset(active_extruder));
        break;
    }
    case OpticalAutocal::ErrorCode::CALIBRATION_FAILED:
        SERIAL_ECHOLN("NOZZLE_AUTOCALIBRATION_FAIL");
        break;
    case OpticalAutocal::ErrorCode::POLARITY_MISMATCH:
        SERIAL_ECHOLN("AUTOCAL_SENSOR_POLARITY_MISMATCH");
        break;
    case OpticalAutocal::ErrorCode::NO_NOZZLE_DETECTED:
        SERIAL_ECHOLN("AUTOCAL_NO_NOZZLE");
        break;
    }
}

void GcodeSuite::M1510()
{
    if (!parser.seen_any()) {
        SERIAL_ECHOLNPGM("Nozzle Calibration Extra Offset\nX: ",
                         OpticalAutocal::nozzle_calibration_extra_offset.x,
                         ", Y: ",
                         OpticalAutocal::nozzle_calibration_extra_offset.y,
                         ", Z: ",
                         OpticalAutocal::nozzle_calibration_extra_offset.z,
                         "\n X correction factor: ",
                         OpticalAutocal::x_offset_factor);
        return;
    }

    if (parser.seen('X'))
        OpticalAutocal::nozzle_calibration_extra_offset.x = parser.value_float();
    if (parser.seen('Y'))
        OpticalAutocal::nozzle_calibration_extra_offset.y = parser.value_float();
    if (parser.seen('Z'))
        OpticalAutocal::nozzle_calibration_extra_offset.z = parser.value_float();
    if (parser.seen('A'))
        OpticalAutocal::x_offset_factor = (1.0f / std::tan(parser.value_float() / 2));
}

void GcodeSuite::M1510_report(bool forReplay)
{
    GcodeSuite::report_heading_etc(forReplay, F("Nozzle Calibration Extra Offset"));
    SERIAL_ECHOLNPGM("M1510 X",
                     OpticalAutocal::nozzle_calibration_extra_offset.x,
                     " Y",
                     OpticalAutocal::nozzle_calibration_extra_offset.y,
                     " Z",
                     OpticalAutocal::nozzle_calibration_extra_offset.z);
}

#endif