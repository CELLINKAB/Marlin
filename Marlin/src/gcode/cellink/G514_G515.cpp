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

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../feature/air_system/pneumatics.h"
#    include "../../feature/bedlevel/bedlevel.h"
#    include "../../feature/guppi_printhead/chantarelle.h"
#    include "../../module/planner.h"
#    include "../gcode.h"
#    include "../parser.h"

/**
 * @brief mixing extrude / pneumatic move
 * 
 */
void GcodeSuite::G514()
{
    uint8_t tool = get_target_extruder_from_command();
    planner.synchronize();
    auto _ = pneumatics::pump.use_pressure();
    pneumatics::apply_mixing_pressure(tool);
    G0_G1(false);
    planner.synchronize();
    pneumatics::release_mixing_pressure(tool);
}

void GcodeSuite::G515()
{
    using namespace pneumatics;
    static constexpr xy_pos_t GRIPPER_ABSOLUTE_XY{140, -45};
    static constexpr float GRIP_Z_HEIGHT = -5.0f;
    static constexpr float RELEASE_Z_HEIGHT = 10.0f;
    static constexpr float GRIP_THRESHOLD = -10.0f;
    static constexpr float RELEASE_THRESHOLD = 0.0f;
    static constexpr float THRESHOLD_HYSTERESIS = 1.0f;

    if (homing_needed_error())
        return;

    TemporaryBedLevelingState scope_leveling(false);

    xyz_pos_t gripper_pos(GRIPPER_ABSOLUTE_XY + hotend_offset[active_extruder]);
    gripper_pos.z = Z_MAX_POS;
    apply_motion_limits(gripper_pos);
    do_blocking_move_to(gripper_pos);

    bool is_releasing = parser.seen('R');

    if (is_releasing) {
        {
            gripper_pos.z = GRIP_Z_HEIGHT;
            do_blocking_move_to(gripper_pos);
            auto _using_pressure = pneumatics::pump.use_pressure();
            set_gripper_valves(GripperState::Release);
            const millis_t timeout = millis() + 8000;
            gripper_pos.z = RELEASE_Z_HEIGHT;
            do_blocking_move_to(gripper_pos);
            while (millis() < timeout && gripper_vacuum.read() < 0)
                idle();
        }
        //Expect pressure down
        set_gripper_valves(GripperState::Close);

        if (float reading = gripper_vacuum.read();
            reading < RELEASE_THRESHOLD - THRESHOLD_HYSTERESIS) {
            SERIAL_ECHOLN("RELEASE_FAILED");
            if (DEBUGGING(INFO))
                SERIAL_ECHOLNPGM("pressure reading: ", reading);
        }
    } else {
        // May need to let out pressure to allow pump to run for grip
        while (tank_pressure.read_avg() >= 75.0f) {
            set_gripper_valves(GripperState::Release);
            WRITE(PRESSURE_VALVE_PUMP_OUT_PIN, PRESSURE_VALVE_OPEN_LEVEL);
            idle();
        }
        set_gripper_valves(GripperState::Grip);
        gripper_pos.z = GRIP_Z_HEIGHT;
        do_blocking_move_to(gripper_pos);
        {
            auto _ = pneumatics::pump.use_suction();
            const millis_t timeout = millis() + SEC_TO_MS(10);
            while (millis() < timeout && gripper_vacuum.read_avg() > GRIP_THRESHOLD) {
                idle();
            }
        }
        set_gripper_valves(GripperState::Close);
        gripper_pos.z = RELEASE_Z_HEIGHT;
        do_blocking_move_to(gripper_pos);

        if (float reading = gripper_vacuum.read(); reading > GRIP_THRESHOLD + THRESHOLD_HYSTERESIS) {
            SERIAL_ECHOLN("GRIP_FAILED");
            if (DEBUGGING(INFO))
                SERIAL_ECHOLNPGM("pressure reading: ", reading);
        }
    }
    gripper_pos.z = Z_MAX_POS;
    do_blocking_move_to(gripper_pos);
}

#endif // FESTO_PNEUMATICS
