
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
    static constexpr xy_pos_t GRIPPER_ABSOLUTE_XY{27, -45};
    static constexpr float GRIPPER_Z_HEIGHT = -5.0f;
    static constexpr float DETECTION_THRESHOLD = 10.0f;

    planner.synchronize();
    auto starting_pos = current_position.copy();
    xy_pos_t gripper_xy(GRIPPER_ABSOLUTE_XY + hotend_offset[active_extruder]);
    do_blocking_move_to(gripper_xy);

    bool is_releasing = parser.seen('R');

    float vacuum_baseline = gripper_vacuum.read_avg();
    SET_SOFT_ENDSTOP_LOOSE(true);
    if (is_releasing) {
        set_gripper_valves(GripperState::Open);
        safe_delay(1000);
    }
    do_blocking_move_to_z(GRIPPER_Z_HEIGHT);
    if (is_releasing) {set_gripper_valves(GripperState::Release); }
    else set_gripper_valves(GripperState::Grip);
    safe_delay(1000);
    do_blocking_move_to_z(starting_pos.z);

    float vacuum_delta = gripper_vacuum.read_avg() - vacuum_baseline;

    if(is_releasing && vacuum_delta < DETECTION_THRESHOLD)
        SERIAL_ERROR_MSG("gripper likely failed to release");
    else if (vacuum_delta > -DETECTION_THRESHOLD)
    {
        SERIAL_ERROR_MSG("gripper likely failed to grip");
    }
    
    do_blocking_move_to(starting_pos);
    
    
}

#endif // FESTO_PNEUMATICS
