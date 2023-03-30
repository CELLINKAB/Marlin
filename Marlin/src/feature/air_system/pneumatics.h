
#pragma once

#include "../../inc/MarlinConfigPre.h"

#include "analog_sensor.h"
#include "pump.h"

namespace pneumatics 
{

void init();

void update();

void set_regulator_pressure(float kPa);
float get_regulator_set_pressure();

enum class GripperState {
    Close,
    Release,
    Grip,
};

void set_gripper_valves(GripperState state);

void apply_mixing_pressure(uint8_t tool);
void release_mixing_pressure(uint8_t tool);


} // namespace pneumatics