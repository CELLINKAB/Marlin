#include "../../inc/MarlinConfig.h"

#if ENABLED(WELLPLATE_EJECT)

#    include "../../feature/bedlevel/bedlevel.h"
#    include "../../feature/door_sensor.h"
#    include "../../module/planner.h"
#    include "../gcode.h"
#    include "../parser.h"

void GcodeSuite::G516()
{
    static constexpr xyz_pos_t EJECT_POS{-139, 260, Z_MAX_POS};
    static constexpr feedRate_t EJECT_FEEDRATE = 100.0f;

    if (homing_needed_error())
        return;

    TemporaryBedLevelingState scope_leveling(false);
    
    xyz_pos_t eject_pos(EJECT_POS + hotend_offset[active_extruder]);
    xyz_pos_t clipped_eject_pos(eject_pos);
    apply_motion_limits(clipped_eject_pos);
    do_blocking_move_to(clipped_eject_pos, EJECT_FEEDRATE);
    soft_endstop._enabled = false;
    do_blocking_move_to(eject_pos, EJECT_FEEDRATE);
    soft_endstop._enabled = true;

    if (!door.read())
        SERIAL_ECHOLN("ERR_DOOR_DID_NOT_OPEN");

    millis_t wait_time = SEC_TO_MS(parser.byteval('S', VESSEL_LOAD_TIMEOUT_SECONDS));
    millis_t timeout = millis() + wait_time;
    wait_for_user_response(wait_time, true);
    if (wait_time && millis() >= timeout)
        SERIAL_ECHOLN("ERR_VESSEL_LOAD_TIMEOUT");

    do_blocking_move_to(clipped_eject_pos, EJECT_FEEDRATE);

    if (door.read())
        SERIAL_ECHOLN("ERR_DOOR_DID_NOT_CLOSE");
}

#endif