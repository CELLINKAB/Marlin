
#include "../../inc/MarlinConfig.h"

#if ENABLED(FESTO_PNEUMATICS)

#    include "../../feature/festo_pneumatics.h"
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
    get_destination_from_command();
    planner.synchronize();
    auto _ = pneumatics::use_pressure();
    pneumatics::apply_mixing_pressure(tool);
    prepare_line_to_destination();
    planner.synchronize();
    pneumatics::release_mixing_pressure(tool);
}

void GcodeSuite::G515()
{
    using namespace pneumatics;
    static constexpr xy_pos_t GRIPPER_ABSOLUTE_XY{135, -45};
    static constexpr float GRIP_Z_HEIGHT = -5.0f;
    static constexpr float RELEASE_Z_HEIGHT = 10.0f;
    static constexpr float GRIP_THRESHOLD = -20.0f;
    static constexpr float RELEASE_THRESHOLD = 0.0f;
    static constexpr float THRESHOLD_HYSTERESIS = 1.0f;

    if (homing_needed_error())
        return;

    xyz_pos_t gripper_xy(GRIPPER_ABSOLUTE_XY + hotend_offset[active_extruder]);
    apply_motion_limits(gripper_xy);
    do_blocking_move_to_z(Z_MAX_POS);
    do_blocking_move_to(gripper_xy);

    bool is_releasing = parser.seen('R');

    if (is_releasing) {
        {
            do_blocking_move_to_z(GRIP_Z_HEIGHT);
            [[maybe_unused]] auto _using_pressure = pneumatics::use_pressure();
            set_gripper_valves(GripperState::Release);
            const millis_t timeout = millis() + 8000;
            current_position.z = RELEASE_Z_HEIGHT;
            planner.buffer_line(current_position, 8.0f);
            while (millis() < timeout && gripper_vacuum.read_avg() < 0)
                idle();
        }
        planner.synchronize();
        //Expect pressure down
        set_gripper_valves(GripperState::Close);

        if (float reading = gripper_vacuum.read();
            reading < RELEASE_THRESHOLD - THRESHOLD_HYSTERESIS) {
            SERIAL_ECHOLN("RELEASE_FAILED");
            if (DEBUGGING(INFO))
                SERIAL_ECHOLNPGM("pressure reading: ", reading);
        }
    } else {
        set_gripper_valves(GripperState::Grip);
        SET_SOFT_ENDSTOP_LOOSE(true);
        do_blocking_move_to_z(GRIP_Z_HEIGHT);
        suck_lid();
        set_gripper_valves(GripperState::Close);
        do_blocking_move_to_z(RELEASE_Z_HEIGHT);
        SET_SOFT_ENDSTOP_LOOSE(false);

        if (float reading = gripper_vacuum.read(); reading > GRIP_THRESHOLD - THRESHOLD_HYSTERESIS) {
            SERIAL_ECHOLN("GRIP_FAILED");
            if (DEBUGGING(INFO))
                SERIAL_ECHOLNPGM("pressure reading: ", reading);
        }
    }
    do_blocking_move_to_z(Z_AFTER_PROBING);
}

#endif // FESTO_PNEUMATICS
