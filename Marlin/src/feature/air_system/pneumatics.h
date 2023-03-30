
#pragma once

#include "../../inc/MarlinConfigPre.h"

#include "analog_sensor.h"
#include "regulator.h"
#include "pump.h"

namespace pneumatics 
{

void init();

void update();

enum class GripperState {
    Close,
    Release,
    Grip,
};

void set_gripper_valves(GripperState state);

void apply_mixing_pressure(uint8_t tool);
void release_mixing_pressure(uint8_t tool);


} // namespace pneumatics