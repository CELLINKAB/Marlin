/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 Cellink [https://github.com/CELLINKAB/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

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

    if (parser.boolval('R')) {
            do_blocking_move_to(clipped_eject_pos, EJECT_FEEDRATE);
            if (!all_axes_trusted())
                process_subcommands_now(F("G28XY"));
    } else {
        apply_motion_limits(clipped_eject_pos);
        do_blocking_move_to(clipped_eject_pos, EJECT_FEEDRATE);
        soft_endstop._enabled = false;
        do_blocking_move_to(eject_pos, EJECT_FEEDRATE);
        soft_endstop._enabled = true;
    }

}

#endif