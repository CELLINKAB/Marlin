#include "../../inc/MarlinConfig.h"

#if ENABLED(WELLPLATE_EJECT)

#    include "../../feature/bedlevel/bedlevel.h"
#    include "../../module/planner.h"
#    include "../gcode.h"
#    include "../parser.h"

void GcodeSuite::G516()
{
    static constexpr xyz_pos_t EJECT_POS{-139, 260, Z_MAX_POS};

    if (homing_needed_error())
        return;

    const bool level_state = planner.leveling_active;
    set_bed_leveling_enabled(false);
    Defer restore_leveling([level_state]() { set_bed_leveling_enabled(level_state); });

    xyz_pos_t eject_pos(EJECT_POS + hotend_offset[active_extruder]);
    toNative(eject_pos);
    xyz_pos_t clipped_eject_pos(eject_pos);
    apply_motion_limits(clipped_eject_pos);
    do_blocking_move_to(clipped_eject_pos);
    do_blocking_move_to(eject_pos);

    static constexpr millis_t WAIT_TIME = SEC_TO_MS(VESSEL_LOAD_TIMEOUT_SECONDS);
    millis_t timeout = millis() + WAIT_TIME;
    wait_for_user_response(WAIT_TIME, true);
    if (WAIT_TIME && millis() > timeout)
        SERIAL_ECHOLN("ERR_VESSEL_LOAD_TIMEOUT");

    do_blocking_move_to(clipped_eject_pos);
}

#endif