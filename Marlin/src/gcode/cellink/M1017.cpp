// Copyright Cellink 2022 - GPLv3

#include "../../inc/MarlinConfig.h"

#if ENABLED(CELLINK_REPORTING)

#    include "../../feature/bedlevel/bedlevel.h"
#    include "../../module/motion.h"
#    include "../../module/stepper.h"
#    include "../gcode.h"

#    include "cellink_reporting.h"

/**
   * @brief report tool offesets
   * 
   */
void GcodeSuite::M1017()
{
    SERIAL_EOL();
    SERIAL_ECHOLN_CELLINK_KV("Active tool", active_extruder);
    for (int8_t e = 0; e < EXTRUDERS; e++)
    {
        SERIAL_ECHO_CELLINK_KV("ToolNum", e);
        SERIAL_ECHO_CELLINK_KV("Calib",
                               false /*isCalibrated(tool)*/); //TODO: implement calibration status per extruder
        SERIAL_ECHO_CELLINK_KV(" X", hotend_offset[e].x);
        SERIAL_ECHO_CELLINK_KV("Y", hotend_offset[e].y);
        SERIAL_ECHO_CELLINK_KV("Z", hotend_offset[e].z);
        SERIAL_EOL();
    }

    SERIAL_ECHOLN("Current pos: ");
    M1015();
    SERIAL_ECHOLN("Current mech pos: ");
    M1016();
    SERIAL_ECHOLN("StepperPos: ");
    SERIAL_ECHO_CELLINK_KV('X', stepper.position(AxisEnum::X_AXIS));
    SERIAL_ECHO_CELLINK_KV('Y', stepper.position(AxisEnum::Y_AXIS));
    SERIAL_ECHO_CELLINK_KV('Z', stepper.position(AxisEnum::Z_AXIS));
    SERIAL_EOL();
    SERIAL_ECHOLN("Logical base offset: ");
    SERIAL_ECHO_CELLINK_KV('X', home_offset.x);
    SERIAL_ECHO_CELLINK_KV('Y', home_offset.y);
    SERIAL_ECHO_CELLINK_KV('Z', home_offset.z);
    SERIAL_ECHOLN("ABL correction: ");
    SERIAL_ECHO_CELLINK_KV('X', 0.0);
    SERIAL_ECHO_CELLINK_KV('Y', 0.0);
    SERIAL_ECHO_CELLINK_KV('Z', ubl.get_z_correction(xy_pos_t{})); // Z offset at 0,0
    SERIAL_EOL();
}

#endif // CELLINK_REPORTING