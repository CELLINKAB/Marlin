
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
    planner.buffer_line(destination, feedrate_mm_s, tool);
    planner.synchronize();
    pneumatics::release_mixing_pressure(tool);
}

void GcodeSuite::G515()
{
    using namespace pneumatics;
    static constexpr xy_pos_t GRIPPER_ABSOLUTE_XY{130, -45};
    static constexpr float GRIP_Z_HEIGHT = -5.0f;
    static constexpr float RELEASE_Z_HEIGHT = 10.0f;
    static constexpr float DETECTION_THRESHOLD = 10.0f;
    static constexpr size_t RELEASE_SECONDS = 5;

    if (homing_needed_error())
        return;

    xyz_pos_t gripper_xy(GRIPPER_ABSOLUTE_XY + hotend_offset[active_extruder]);
    apply_motion_limits(gripper_xy);
    do_blocking_move_to_z(Z_AFTER_PROBING);
    do_blocking_move_to(gripper_xy);

    bool is_releasing = parser.seen('R');

    float vacuum_baseline = gripper_vacuum.read_avg();
    if (is_releasing) {
        do_blocking_move_to_z(RELEASE_Z_HEIGHT);
        set_gripper_valves(GripperState::Release);
        for (size_t seconds = RELEASE_SECONDS; seconds > 0; --seconds) {
            idle();
            safe_delay(1000);
        }
        //Expect pressure down
        float vacuum_delta =   vacuum_baseline -gripper_vacuum.read_avg();
        if (DEBUGGING(INFO)) {
            SERIAL_ECHOLNPAIR_F("vacuum_baseline:", vacuum_baseline);
            SERIAL_ECHOLNPAIR_F("vacuum_delta:", vacuum_delta); 
        }
        if (vacuum_delta < DETECTION_THRESHOLD) {
            SERIAL_ECHOLN("RELEASE_FAILED");

        }
    } else {
        set_gripper_valves(GripperState::Open);
        SET_SOFT_ENDSTOP_LOOSE(true);
        do_blocking_move_to_z(GRIP_Z_HEIGHT);
        set_gripper_valves(GripperState::Grip);
        do_blocking_move_to_z(RELEASE_Z_HEIGHT);
        SET_SOFT_ENDSTOP_LOOSE(false);
        // Expect pressure up
        float vacuum_delta = gripper_vacuum.read_avg() -vacuum_baseline;
        if (DEBUGGING(INFO)) {
            SERIAL_ECHOLNPAIR_F("vacuum_baseline:", vacuum_baseline);
            SERIAL_ECHOLNPAIR_F("vacuum_delta:", vacuum_delta); 
        }
        if (vacuum_delta < DETECTION_THRESHOLD) {
            SERIAL_ECHOLN("GRIP_FAILED");
        }
    }
    do_blocking_move_to_z(Z_AFTER_PROBING);
}

#endif // FESTO_PNEUMATICS
