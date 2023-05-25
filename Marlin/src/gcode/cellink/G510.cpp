#include "../../inc/MarlinConfig.h"

#if ENABLED(OPTICAL_AUTOCAL)

#    include "../../feature/optical_autocal.h"
#    include "../../module/motion.h"
#    include "../gcode.h"

#    include <stdio.h>

xyz_pos_t OpticalAutocal::nozzle_calibration_extra_offset{};

static void update_offset(const xyz_pos_t& offset)
{
    position_shift
    set_home_offset(AxisEnum::X_AXIS, -(offset.x + OpticalAutocal::nozzle_calibration_extra_offset.x));
    set_home_offset(AxisEnum::Y_AXIS, offset.y + OpticalAutocal::nozzle_calibration_extra_offset.y);
    set_home_offset(AxisEnum::Z_AXIS, offset.z + OpticalAutocal::nozzle_calibration_extra_offset.z);
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

    const auto feedrate = parser.feedrateval('F', 25.0f);

    if (parser.seen('D')) {
        optical_autocal.test(48, start_pos, feedrate);
        return;
    }

    // reset calibration so results are consistent
    optical_autocal.reset(active_extruder);
    update_offset(optical_autocal.offset(active_extruder));

    switch (optical_autocal.full_autocal_routine(start_pos, feedrate)) {
    case OpticalAutocal::ErrorCode::SANITY_CHECK_FAILED:
        SERIAL_ECHOLN("AUTOCAL_SANITY_CHECK_FAIL");
        [[fallthrough]];
    case OpticalAutocal::ErrorCode::OK: {
        update_offset(optical_autocal.offset(active_extruder));
        xy_pos_t origin{0, 0};
        toNative(origin);
        do_blocking_move_to_xy(origin);
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
                         OpticalAutocal::nozzle_calibration_extra_offset.z);
        return;
    }

    if (parser.seen('X'))
        OpticalAutocal::nozzle_calibration_extra_offset.x = parser.value_float();
    if (parser.seen('Y'))
        OpticalAutocal::nozzle_calibration_extra_offset.y = parser.value_float();
    if (parser.seen('Z'))
        OpticalAutocal::nozzle_calibration_extra_offset.z = parser.value_float();
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